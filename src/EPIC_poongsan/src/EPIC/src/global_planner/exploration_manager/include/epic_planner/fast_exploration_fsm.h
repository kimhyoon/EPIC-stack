/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2023-12-21 21:31:51
 * @LastEditTime: 2024-03-06 10:28:56
 * @Description:
 * @
 * @Copyright (c) 2023 by ning-zelin, All Rights Reserved.
 */

#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <epic_planner/fast_exploration_manager.h>
#include <iostream>
#include <memory>
#include <message_filters/simple_filter.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pointcloud_topo/graph_visualizer.hpp>
#include <quadrotor_msgs/TakeoffLand.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <epic_planner/GoalService.h>
#include <epic_planner/event_logger.h>
#include <std_srvs/Trigger.h>

using Eigen::Vector3d;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
class FastPlannerManager;
class FastExplorationManager;
class PlanningVisualization;
struct FSMParam;
struct FSMData;

// MAVROS can switch its odometry header from ROS time to an FCU clock domain
// after time synchronisation locks. ApproximateTime then stops pairing odom
// with the lidar cloud even though both messages still arrive normally. Keep
// valid, near-ROS-time stamps untouched, but put grossly skewed stamps back in
// the current ROS clock domain before they reach the synchronizer.
class StampNormalizingOdomFilter
    : public message_filters::SimpleFilter<nav_msgs::Odometry> {
public:
  void subscribe(ros::NodeHandle &nh, const string &topic, uint32_t queue_size,
                 double max_stamp_skew) {
    max_stamp_skew_ = std::max(0.0, max_stamp_skew);
    subscriber_ = nh.subscribe(topic, queue_size,
                               &StampNormalizingOdomFilter::callback, this);
  }

private:
  void callback(const nav_msgs::OdometryConstPtr &message) {
    const ros::Time now = ros::Time::now();
    const double skew = (message->header.stamp - now).toSec();
    if (!message->header.stamp.isZero() && std::isfinite(skew) &&
        std::abs(skew) <= max_stamp_skew_) {
      signalMessage(message);
      return;
    }

    nav_msgs::OdometryPtr normalized(new nav_msgs::Odometry(*message));
    normalized->header.stamp = now;
    ROS_WARN_THROTTLE(
        2.0,
        "[OdomStamp] normalizing odometry stamp for cloud sync: "
        "source=%.3f now=%.3f skew=%.3f s (limit %.3f s)",
        message->header.stamp.toSec(), now.toSec(), skew, max_stamp_skew_);
    signalMessage(normalized);
  }

  ros::Subscriber subscriber_;
  double max_stamp_skew_ = 5.0;
};

// NOTE: TAKEOFF_HOVER/LANDED are appended at the END so existing enum indices
// (used as indices into fd_->state_str_) stay unchanged.
// LANDED: LAND(OFFBOARD 하강) 후 착지+disarm 이 확인된 최종 상태.
//         record_on_goal 이 이 상태를 보고 녹화를 마감한다.
// EARLY_FINISH: 탐사가 너무 짧게 끝났을 때(누적 이동거리 < thresh), 현재 odom
//               node 에서 가장 먼 도달 가능 topology node 까지의 기존 graph path 를
//               global tour 로 설치한 뒤 PLAN_TRAJ_EXP 로 돌아간다.
// YAW_ROTATE_INIT: 이륙 호버 지점을 유지한 채 제자리에서 360도 회전해 주변을 한
//         바퀴 관측한 뒤 탐사를 시작한다. 이륙 직후 맵이 비어 frontier 가 0개로
//         보이는 상황에 대한 사전 예방이다 (EARLY_FINISH 는 같은 문제의 사후 구제).
// PILOT_OVERRIDE: 비행 중 PX4 가 OFFBOARD 를 벗어나면 조종자/비행 컨트롤러가
//         소유권을 가져간 것으로 보고 planner 를 영구 동결하는 terminal 상태.
//         새 enum 도 END 에 추가해 기존 인덱스를 건드리지 않는다.
enum EXPL_STATE { INIT, WAIT_TRIGGER, PLAN_TRAJ_EXP, PLAN_TRAJ_RTH, CAUTION, EXEC_TRAJ, FINISH, LAND, TAKEOFF_HOVER, LANDED, EARLY_FINISH, YAW_ROTATE_INIT, PILOT_OVERRIDE, MAP_REBUILD };

class FastExplorationFSM {
private:
  /* planning utils */
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<FastExplorationManager> expl_manager_;
  shared_ptr<PlanningVisualization> visualization_;

  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  EXPL_STATE state_;

  // CAUTION is an owning FSM with two explicit phases.  While an escape is
  // executing, no planning callback may replace it or infer completion from a
  // transient clearance sample.
  enum CAUTION_PHASE { CAUTION_IDLE, CAUTION_EXEC_ESCAPE };
  CAUTION_PHASE caution_phase_ = CAUTION_IDLE;
  int caution_escape_traj_id_ = 0;
  ros::Time caution_enter_time_;
  ros::Time caution_last_attempt_time_;
  int caution_escape_fail_count_ = 0;
  bool caution_map_reset_enable_ = true;
  double caution_map_reset_timeout_ = 6.0;
  int caution_map_reset_failure_count_ = 10;
  double caution_retry_period_ = 0.5;

  // A map epoch reset is a bounded recovery state, not an instantaneous jump
  // back into planning. Hold the vehicle until fresh synchronized scans have
  // repopulated occupancy and reconnected the live odom node to topology.
  ros::Time map_rebuild_start_time_;
  uint64_t map_rebuild_start_update_seq_ = 0;
  Eigen::Vector3d map_rebuild_hold_pos_ = Eigen::Vector3d::Zero();
  double map_rebuild_hold_yaw_ = 0.0;
  EXPL_STATE map_rebuild_resume_state_ = PLAN_TRAJ_EXP;
  double map_rebuild_min_duration_ = 1.0;
  int map_rebuild_min_scans_ = 10;
  int map_reset_count_ = 0;
  ros::Time map_rebuild_last_status_time_;

  // RTH failures are counted in a short rolling window. Sparse failures age
  // out; a tight replan/collision loop causes exactly one map rebuild.
  std::deque<ros::Time> rth_failure_times_;
  ros::Time last_rth_failure_time_;
  bool rth_map_reset_enable_ = true;
  double rth_failure_window_ = 3.0;
  double rth_failure_min_interval_ = 0.25;
  int rth_map_reset_failure_count_ = 5;

  bool classic_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, global_path_update_timer_;
  ros::Subscriber trigger_sub_, map_update_sub_, battary_sub_, avoid_flag_sub_;
  ros::Publisher stop_pub_, new_pub_, replan_pub_, poly_traj_pub_, heartbeat_pub_, time_cost_pub_, poly_yaw_traj_pub_, static_pub_, state_pub_,
  land_pub_, rth_metrics_pub_, hover_cmd_pub_;
  ros::Publisher early_finish_state_pub_;
  // Latched asynchronous result for /srv_reset_map. Operators and bag tests
  // can distinguish STARTED/WAITING/COMPLETE without scraping console logs.
  ros::Publisher map_rebuild_status_pub_;
  // exploration debug HUD: text marker (rviz) + string (logging/bag)
  // + 기계 파싱용 key=value 진단 (record_on_goal 이 epic.log 로 기록)
  ros::Publisher diag_pub_, diag_str_pub_, diag_kv_pub_;
  double last_plan_ms_ = 0.0;  // 마지막 global plan 총 소요시간 [ms] (HUD 표시용)
  void publishExplDiag();  // 클러스터/뷰포인트 수 + 실패 사유를 rviz/로그로 발행

  /* structured flight-event logging (see event_logger.h) */
  EventLogger elog_;
  std::string local_reason_;          // 마지막 로컬 계획 실패 사유 (성공 시 clear)
  std::vector<std::string> param_lines_; // PARAM 이벤트 라인 캐시 (트리거 시 재발행)
  void logParamsEvents(bool force);   // 주요 파라미터를 이벤트+latched 토픽으로 덤프
  // 미션 시작 공용 진입점 (rviz goal/waypoints 토픽/서비스가 모두 여길 탐).
  // WAIT_TRIGGER 가 아니면 false.
  bool startMission(const std::string &source);
  // rviz 없이 터미널 한 줄로 시작: rosservice call /srv_start
  ros::ServiceServer srv_start_;
  bool startServiceCallback(std_srvs::Trigger::Request &req,
                            std_srvs::Trigger::Response &res);
  // 비행 중 즉시 원점(이륙지점) 복귀+착륙: rosservice call /srv_rth
  // (좌표 지정 이동은 /srv_goto 로 분리 — GoalService 타입 유지)
  ros::ServiceServer srv_rth_home_;
  bool rthServiceCallback(std_srvs::Trigger::Request &req,
                          std_srvs::Trigger::Response &res);
  ros::ServiceServer srv_early_finish_;
  bool earlyFinishServiceCallback(std_srvs::Trigger::Request &req,
                                  std_srvs::Trigger::Response &res);
  // Operator/test hook. Automatic CAUTION/RTH escalation calls the same
  // implementation, so bag validation exercises the production reset path.
  ros::ServiceServer srv_reset_map_;
  bool resetMapServiceCallback(std_srvs::Trigger::Request &req,
                               std_srvs::Trigger::Response &res);
  bool beginMapRebuild(const std::string &reason,
                       bool force_return_home = false);
  void publishMapRebuildStatus(const std::string &phase,
                               const std::string &detail);
  bool noteRthFailure(const std::string &reason);
  void clearRthFailures();
  void logGlobalPlanEvent(int res, double t_ms); // GLOBAL 이벤트 공통 발행
  bool verbose_console_ = false;      // true 면 기존 타이밍 cout/INFO 유지
  /* [feature: astar-profile] A* 탐색 소요시간 분포 발행.
     parallel_astar/*_timeout 을 올릴지 판단하는 근거 (/planning/timing/astar_profile). */
  ros::Publisher astar_profile_pub_;
  double astar_profile_period_ = 5.0;   // [s] 벽시계 기준 발행 주기
  double astar_conn_timeout_ms_ = 0.0, astar_insert_timeout_ms_ = 0.0;

  /* stuck watchdog: 미션 상태에서 장시간 무이동 감지 (이벤트만, 자동회복 아님) */
  Eigen::Vector3d stuck_ref_pos_ = Eigen::Vector3d::Zero();
  ros::Time stuck_ref_t_;
  /* AVOID 이벤트 에지용 */
  ros::Time avoid_on_t_;
  /* PX4 mode/armed 변화 감지용 */
  bool px4_seen_ = false;
  /* battery 이벤트 */
  double battery_warn_voltage_ = 21.0;
  /* reactive local avoidance 마스터 스위치 (real.yaml local_avoidance/enable) */
  bool avoidance_enabled_ = true;
  ros::ServiceServer srv_goal_;
  
  // Global planning timing publishers
  ros::Publisher update_topo_skeleton_cost_pub_, update_odom_vertex_cost_pub_, vp_cluster_cost_pub_, 
                 remove_unreachable_cost_pub_, select_vp_cost_pub_, insert_viewpoint_cost_pub_,
                 calculate_tsp_cost_pub_, lkh_solver_cost_pub_, call_planner_cost_pub_,
                 ikd_tree_insert_cost_pub_, update_frontier_clusters_cost_pub_,
                 fast_searcher_search_cost_pub_, bubble_astar_search_cost_pub_;
  double total_time_;

  /*cloud odom callback*/
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicyCloudOdom;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyCloudOdom>> SynchronizerCloudOdom;

  // Keep the subscribers alive until after the synchronizer disconnects from
  // their signals. Members are destroyed in reverse declaration order, so the
  // synchronizer must be declared last. The old order destroyed odom_sub_ and
  // cloud_sub_ first; then Synchronizer::~Synchronizer() tried to remove its
  // callbacks through already-destroyed Signal1 mutexes and aborted with
  // boost::lock_error whenever a same-name shutdown unwound main().
  shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
  shared_ptr<StampNormalizingOdomFilter> odom_sub_;
  SynchronizerCloudOdom sync_cloud_odom_;
  void CloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr &msg, const nav_msgs::Odometry::ConstPtr &odom_);

  /* goal-directed navigation */
  Eigen::Vector4d goal_rth_;  // x, y, z, yaw
  bool has_goal_rth_;
  double goal_tolerance_;

  /* local planning rate control */
  double local_planning_max_hz_;
  double local_planning_min_period_;

  /* reactive-avoidance hand-off (Phase 2): re-anchor planning to the current
     pose while the reactive layer is avoiding, so the trajectory handed back to
     PX4 on release starts where the drone actually is (no snap-back). */
  int    avoid_flag_ = 0;            // last /FSM_flag_avoidance value (1 = obstacle close)
  bool   have_avoid_flag_ = false;
  bool   avoiding_prev_ = false;     // previous-tick avoidance state (for 1->0 edge)
  ros::Time last_avoid_flag_stamp_;
  double avoid_flag_timeout_ = 0.5;  // [s] treat the flag as stale (silent) after this

  /* takeoff & hover-before-explore (real flight) */
  Eigen::Vector3d takeoff_anchor_ = Eigen::Vector3d::Zero();  // (x,y,target_z) held during climb
  double takeoff_yaw_ = 0.0;                                  // heading held during climb
  ros::Time hover_enter_time_;                                // when TAKEOFF_HOVER began
  ros::Time hover_stable_since_;                              // when the drone became reached+stable (0 = not yet)

  /* startup warmup: don't treat NO_FRONTIER as "exploration finished" before the
     map/frontiers are actually ready (e.g. cloud not published yet at trigger). */
  bool   frontiers_ever_seen_ = false;       // latched once the planner first succeeds
  ros::Time explore_start_time_;             // first plan attempt (0 = unset)
  double explore_warmup_timeout_ = 5.0;      // [s] after this, an empty map may still finish

  /* auto return-to-home + land after exploration finishes (real flight).
     FINISH -> hover briefly at the last cmd pose (stays OFFBOARD) -> RTH to the
     takeoff point -> when within xy tol of home, descend on OFFBOARD setpoints
     and force-disarm on touchdown (LAND). */
  bool   auto_rth_land_ = true;              // master enable
  bool   traj_server_owns_finish_cmd_ = false;  // release /position_cmd to traj_server in FINISH
  double finish_hover_duration_ = 3.0;       // [s] hover at last cmd pose before returning
  double rth_land_xy_tol_ = 0.3;             // [m] xy proximity to home that triggers landing
  bool   explore_finished_ = false;          // latched only when EXPLORATION ends (not service-RTH)
  bool   returning_home_ = false;            // auto RTH-then-land in progress (routes RTH->LAND)
  ros::Time finish_hover_start_;             // when the FINISH hover began (0 = not yet)
  Eigen::Vector3d finish_hover_pos_ = Eigen::Vector3d::Zero();  // snapshot of last cmd pose
  double finish_hover_yaw_ = 0.0;
  ros::Subscriber    mavros_state_sub_;      // /mavros/state (armed/mode 관측)
  mavros_msgs::State px4_state_;

  /* ================= [offboard landing] PX4 AUTO.LAND 대신 직접 하강 =========
     배경: AUTO.LAND 착륙 시 "옆으로 조금씩 샌다"는 현장 보고. 증상은 실재하나
     원인은 미확정이다. 후보와 판정 상태는 Update_log-0810.txt [1] 에 정리했다.
       A) MPC_TILTMAX_LND(12도) 착륙 틸트 제한 — 파라미터는 실재하나, v1.13 소스상
          적용 조건이 TakeoffState 라 하강 중에는 45도일 가능성. 판정 보류.
       B) 위치 추정 무효화 -> FlightTaskDescend(xy 제어 없음) — 유력. 이 원인이면
          이 변경으로 고쳐지지 않는다(localization 과제).
       C) Navigator 의 global 경유,  D) xy 셋포인트가 진입 시점에 고정.

     원인과 무관하게 이 구현이 얻는 것:
       1. xy 를 "착륙을 시작한 자리"가 아니라 기억해 둔 이륙 좌표에 계속 묶는다(D 해소).
       2. 하강 속도를 우리가 쥔다. AUTO.LAND 는 MPC_LAND_SPEED(기본 0.7m/s) 고정.
       3. 접지 즉시 강제 disarm — 모터가 지면에서 도는 시간을 줄인다.
       4. Navigator/global 경로를 지나지 않는다(C 해소).
       5. 판정 시점과 근거가 전부 로그로 남는다.

     명령은 전부 위치 셋포인트다 — xy 는 이륙 좌표 고정, z 는 LAND 진입 시점 z 에서
     land_ramp_rate_ 기울기로 내려가는 램프. 즉 PX4 입장에서는 평상시 비행과 완전히 같은
     종류의 명령이고, 새로 쓰는 기능이 없다.
       · z 를 절대 목표(지면+0.1)로 주지 않고 램프로 주는 이유: 절대 z 를 명령하면
         추정 바이어스가 그대로 지면 오차가 되고(0803 에서 PX4 EKF 와 dz 최대 1.4m),
         하강 속도도 MPC_Z_VEL_MAX_DN 에 뺏긴다. 램프는 상대량이라 바이어스가
         상쇄되고 기울기가 곧 하강 속도다.
       · 속도 셋포인트를 쓰지 않는 이유: px4_ctrl_bridge 는 모든 경로에서
         IGNORE_VX|VY|VZ 로 속도 필드를 꺼 두고 있다. 이 기체에서 한 번도 쓰인 적이
         없는 경로라, 검증 없이 착륙에 쓸 이유가 없다. 램프 기울기로 충분하다.

     접지 판정은 단일 고도 트립으로 하지 않는다. 고도만 보면 위 바이어스에 그대로
     노출되므로, "지면이 기체를 받치고 있다"는 물리적 증거를 AND 로 묶는다:
       (1) 추정 AGL < land_touch_alt_       (상대고도, ToF 가 있으면 ToF 우선)
       (2) 하강을 명령했는데 |vz| 가 죽어 있음  <- 바이어스와 무관한 핵심 증거
       (3) 위 둘이 land_touch_hold_ 동안 지속
     또는 PX4 자체 land detector(/mavros/extended_state 의 ON_GROUND)가 뜨면 즉시.
     AUTO.LAND 로의 전환/폴백 경로는 없다. 착륙 중 문제가 생기면 모드를 바꾸는
     대신 하강 명령과 강제 disarm 재시도를 유지하며 ERROR 로 조종자 개입을
     요구한다. 그 아래는 PX4 자체 failsafe 소관이다. */
  enum LAND_PHASE {
    LAND_PHASE_IDLE,      // 아직 시작 안 함
    LAND_PHASE_DESCEND,   // vz 음수로 하강 중
    LAND_PHASE_DISARM     // 접지 확인, 강제 disarm 요청 중
  };
  // 하강 속도는 전 구간 동일하다. 단계를 두면 전환 지점에서 vz 명령이 계단처럼
  // 튀는데, 착륙을 몇 초 빨리 끝내자고 감수할 이득이 아니다. 느리게 내려간다.
  // 램프의 기울기. 위치 셋포인트를 이 비율로 낮추므로 곧 하강 속도가 된다.
  // "속도 셋포인트"가 아니다 — PX4 는 이 값을 보지 못하고 위치만 받는다.
  double land_ramp_rate_        = 0.2;       // [m/s] 램프 기울기 = 하강 속도
  // 0.13: 0806 실비행 bag 8개에서 접지 순간 z 드리프트가 최대 +0.087m 였다(전부 양수
  // = odom 이 실제보다 높게 읽음). 0.10 이면 여유가 13mm 뿐이라 다음 비행에서 조금만
  // 더 드리프트하면 게이트가 영영 안 걸린다.
  double land_touch_alt_    = 0.13;      // [m] 접지 판정 AGL 문턱
  double land_touch_hold_   = 0.30;      // [s] 접지 조건 지속 시간
  // 아래 둘은 경고 임계값일 뿐 하강을 중단시키지 않는다 (중단할 곳이 없다).
  // land_xy_err_warn_ 이 재는 것은 "절대 위치가 얼마나 틀렸나"가 아니라 위치제어기의
  // 추종 오차다 — 셋포인트도 측정도 같은 odom 프레임이므로, 이 값이 크다는 것은
  // 제어기가 앵커를 못 잡고 있다(틸트 포화/외란)는 뜻이다. 반대로 LIO xy 자체가
  // 드리프트하면 기체가 실제로 밀려도 이 값은 0 에 가깝다 — 그건 못 잡는다.
  // (실내에 GPS 가 없고 ToF 는 z 전용이라 xy 절대위치를 검증할 독립 소스가 없다.)
  double land_xy_err_warn_  = 0.80;      // [m] 추종 오차가 이보다 크면 ERROR 로그
  double land_timeout_      = 25.0;      // [s] 하강이 이보다 길어지면 ERROR 로그

  /* 램프 anti-windup.
     위치 램프는 속도 명령과 달리 기체가 뒤처져도 혼자 계속 내려간다. 지면효과로
     추종이 밀리면 셋포인트가 달아나 과하게 밀어붙이게 되므로, 실제 z 보다 이만큼
     아래로는 못 가게 묶는다. */
  double land_ramp_lead_max_ = 0.30;     // [m] 램프가 실제 z 보다 앞설 수 있는 한계

  LAND_PHASE      land_phase_ = LAND_PHASE_IDLE;
  ros::Time       land_enter_time_;      // LAND 진입 시각
  ros::Time       land_touch_since_;     // 접지 조건이 연속으로 성립하기 시작한 시각 (0 = 미성립)
  ros::Time       land_disarm_since_;    // disarm 요청을 시작한 시각
  ros::Time       land_last_req_;        // disarm/set_mode 재요청 스로틀
  Eigen::Vector3d land_xy_anchor_ = Eigen::Vector3d::Zero();  // 붙잡을 홈 xy (+ 램프용 z)
  double          land_yaw_ = 0.0;
  double          land_z_ramp_ = 0.0;    // 적분한 z 위치 목표 (하강 램프)
  ros::Time       land_last_tick_;       // z 램프 적분용
  double          land_ground_z_ = 0.0;  // 이륙 시점 지면의 LIO z (AGL 기준선)
  bool            land_ground_z_valid_ = false;
  bool            land_bias_logged_ = false;
  // 접지가 확인된 뒤인가. 'touchdown detected' 이후를 뜻한다.
  bool            land_touchdown_confirmed_ = false;

  uint8_t         px4_landed_state_ = 0;    // mavros ExtendedState::LANDED_STATE_*
  bool            px4_ext_seen_ = false;

  ros::ServiceClient command_client_;    // /mavros/cmd/command  (강제 disarm)
  ros::Subscriber    mavros_ext_state_sub_;  // /mavros/extended_state (PX4 land detector)

  /* [YAW_ROTATE_INIT] 이륙 후 제자리 360도 초기 관측 회전.
     완료 판정은 odom yaw 누적으로만 한다 — 명령만 적분하면 실제로 안 돌았는데
     돌았다고 판단할 수 있다. 의도적으로 타임아웃이 없다: 회전이 확인되지 않으면
     탐사를 시작하지 않고 제자리 호버로 남는 편이 안전하다. */
  bool   yaw_rotate_init_enable_ = true;
  double yaw_rotate_init_rate_ = 0.5;    // [rad/s] 회전 각속도
  double yaw_rotate_accum_ = 0.0;        // 실제(odom) 누적 회전량 [rad]
  double yaw_rotate_cmd_yaw_ = 0.0;      // 명령 yaw (매 틱 적분)
  float  yaw_rotate_last_yaw_ = 0.0f;    // 직전 odom yaw (증분 계산용)
  bool   yaw_rotate_valid_ = false;      // 직전 odom yaw 유효 여부
  ros::Publisher yaw_rotate_state_pub_;  // /planning/yaw_rotate_init (진행률)

  /* 이동량 지표 두 개. FINISH 가 "정말 다 봐서" 끝난 건지 "아직 못 돌아다녀서"
     끝난 건지 가르는 근거는 max_displacement_ (탐사 시작점 기준 **최대** 직선
     이탈거리, 단조증가) 다. 경로 적분(traveled_distance_)은 리플랜 플래핑이나
     근거리 셔플만으로도 임계값을 소진해 구제가 죽는 문제가 있어 판정에서 뺐고
     진단용으로만 남긴다. "멀리 갔다 원점 복귀 = 변위 0" 오판은 현재 변위가 아닌
     최대 변위를 쓰므로 성립하지 않는다 (단조증가라 복귀해도 줄지 않는다). */
  double traveled_distance_ = 0.0;                                  // [m] 탐사 시작 후 누적 (진단용)
  Eigen::Vector3f last_traveled_pos_ = Eigen::Vector3f::Zero();     // 직전 적분 기준점
  bool   traveled_valid_ = false;                                   // 기준점이 유효한가
  double max_displacement_ = 0.0;                                   // [m] expl_origin_ 기준 최대 직선거리
  Eigen::Vector3f expl_origin_ = Eigen::Vector3f::Zero();           // 탐사 시작점 (max/EFP 랭킹 공용 기준점)

  /* EARLY_FINISH: 조기 종료로 판단되면 "가장 먼 도달 가능 topology node" 를
     EFP(early-finish point)로 잡고, **도달할 때까지 죽지 않는 목표**로 관리한다.
     EARLY_FINISH 상태 진입은 예전처럼 1회뿐이고, 바뀌는 건 EFP 가 생긴 "이후"의
     수명 관리다. EFP 는 매 전역계획 주기마다 TSP 후보 목록에 한 개 더 얹힌다
     (fast_exploration_manager.cpp planGlobalPath). 그 결과 아래 셋이 FSM 상태
     분기 없이 자연히 창발한다:
       1) 도달  : reach_tol 안에 들어오면 후보에서 빠지고 원래 흐름으로 복귀
                  (남은 vp 소진 -> NO_FRONTIER -> FINISH -> RTH)
       2) 새 vp : viewpoints = {실제 vp들} U {EFP} 라서 LKH 가 순수 거리로 순서만
                  정한다. EFP 를 버리는 분기가 없다
       3) 끊김  : 매 주기 odom_node 와의 연결을 확인하고, 끊겼으면 그 자리에서
                  "도달 가능 노드 중 옛 EFP 좌표에 최근접" 노드로 재바인딩
     "EFP 에 도달해야만 RTH" 는 별도 게이트 코드가 아니라, EFP 가 살아있는 한
     viewpoints 가 비지 않아 NO_FRONTIER 가 안 난다는 사실에서 나온다. */
  bool   early_finish_enable_ = true;
  int    early_finish_max_retry_ = 1;          // 재시도 횟수 상한 (무한루프 방지 래치)
  // [m] 자동 구제의 유일한 판정: EFP 후보의 원점거리가 max_displacement_ + 이 값을
  // 넘어야 probe 를 보낸다. 이미 가본 범위 안이면 관측 이득이 없기 때문.
  // (예전 1단 거리 게이트 early_finish_dist_thresh 는 폐지 — 절대 임계는 환경
  // 규모를 몰라 3m 이상 진출한 조기 종료를 구제하지 못했다.)
  double early_finish_probe_min_gain_ = 1.0;
  // [m] 비행경로(history odom nodes)에서 이 반경 안의 노드는 "방문한 곳"으로 보고
  // EFP 후보에서 제외한다. 0 이하면 방문 필터 비활성.
  double early_finish_visited_radius_ = 1.5;
  // [m] EFP 도착 판정 (순수 3D 유클리드, yaw 조건 없음).
  // 파라미터는 ViewpointManager/vp_reached_pos_tol — frontier viewpoint 도달
  // 판정과 같은 값을 공유한다 (EFP 도 결국 TSP 의 viewpoint 후보 하나다).
  double early_finish_reach_tol_ = 0.6;
  int    early_finish_count_ = 0;              // 자동 재시도 횟수
  bool   early_finish_active_ = false;         // EFP outstanding (도달 전까지 true)
  bool   early_finish_force_requested_ = false;
  bool   early_finish_forced_attempt_ = false;
  double early_finish_graph_distance_ = 0.0;   // odom -> EFP 그래프 거리[m] (매 주기 갱신)
  TopoNode::Ptr early_finish_node_;            // 현재 EFP 에 바인딩된 실제 topo node
  // EFP 좌표. 노드 포인터가 그래프에서 사라져도 이 좌표는 남아야 한다 —
  // 재바인딩이 "옛 EFP 좌표에 최근접" 기준이라 이게 유일한 기준점이다.
  Eigen::Vector3f early_finish_target_pos_ = Eigen::Vector3f::Zero();
  float  early_finish_yaw_ = 0.0f;             // EFP 로 향할 때 바라볼 방향 (바인딩 시점 진행방향)
  int    early_finish_rebind_count_ = 0;       // 재바인딩 횟수 (진단용)
  string early_finish_status_ = "IDLE";

  /* FINISH-time amnesty: global_planning.cpp 의 게이트(`if (!is_reachable_)
     continue;`) 때문에 클러스터의 is_reachable_ 은 한 번 false 가 되면 재평가
     경로가 없는 단방향 래치다. FINISH 진입 직전 딱 한 번 전부 사면하고 전역계획을
     재시도해, 실제로는 도달 가능한 프론티어가 남았는데 조기 종료되는 것을 막는다.
     사면 직후 라운드는 모든 클러스터가 is_reachable_=true 상태로 평가되므로
     vp_stats_.unreachable_pre 가 0 이 된다. 따라서 prev_unreachable > 0 게이트만으로
     "직전 사면 결과에 대고 연달아 또 사면하는" 경우가 자연히 차단되므로, 별도의
     시간 기반 rate limit 은 두지 않는다 (retryWithUnreachableAmnesty() 주석 참조). */
  bool   unreachable_amnesty_enable_ = true;

  /* helper functions */
  // 기체 중심 planning_box 안인가. planning_box_num_<=0 이면 제한 없음(true).
  bool insidePlanningBox(const Eigen::Vector3f &p) const;
  // odom_node_ 에서 Dijkstra 를 한 번 흘려 "도달 가능한 노드 -> 그래프 거리[m]"
  // 를 통째로 반환한다. EFP 선정(가장 먼 노드)과 매 주기 연결성 확인/재바인딩이
  // 모두 이 한 함수를 공유한다.
  bool computeReachableNodes(std::unordered_map<TopoNode::Ptr, double> &dist_out);
  // p 가 비행경로(history odom nodes)의 early_finish_visited_radius_ 안인가.
  bool isNearFlownPath(const Eigen::Vector3f &p) const;
  // require_gain=true (자동 구제): 미방문 후보 중 원점거리 최대 노드를 고르되,
  // 그 거리가 max_displacement_ + probe_min_gain 을 넘어야 성공. false (강제
  // /srv_early_finish): gain 조건 없음, 미방문 후보가 없으면 방문 필터도 푼다.
  bool selectFarthestReachableNode(TopoNode::Ptr &node_out, double &dist_out,
                                   bool require_gain);
  bool installEarlyFinishProbe(const TopoNode::Ptr &node, double graph_distance);
  // 매 전역계획 직전: 도달 판정 -> 연결성 확인/재바인딩 -> planner 로 값 push.
  void updateEarlyFinishProbe();
  // 도달 판정만 (FSM 틱마다 상시 감시). 도달하면 EFP 를 소멸시킨다.
  void checkEarlyFinishReached();
  // 현재 EFP 를 ed_(planGlobalPath 가 읽는다) 와 rviz 마커로 밀어넣는다.
  void pushEarlyFinishProbe();
  void clearEarlyFinishPath(const string &status);
  void publishEarlyFinishStatus(const string &status,
                                const string &detail = "");
  bool explorationReallyFinished();          // false during startup warmup, true once frontiers seen / timeout
  // FINISH 진입 직전 마지막 방어선: 클러스터의 is_reachable_ 래치를 전부 풀고
  // 전역계획을 한 번 더 시도한다. true 를 반환하면 재계획이 성공(≠NO_FRONTIER)한
  // 것이므로 호출부는 FINISH 로 가지 말고 탐사를 계속해야 한다.
  bool retryWithUnreachableAmnesty();
  void mavrosStateCallback(const mavros_msgs::State::ConstPtr &msg);
  // yaw_dot 을 실어 보내면 px4_ctrl_bridge(use_yawrate=true)가 yawrate 로 회전시킨다.
  void pubHoldCmd(const Eigen::Vector3d &p, double yaw, double yaw_dot = 0.0);

  /* [offboard landing] LAND 상태 본체. 하강 -> 접지 -> 강제 disarm. */
  void runOffboardLanding();
  /* 하강 셋포인트 발행. 실기: /land_setpoint (xy 위치 + z 속도 혼합 마스크).
     시뮬: /position_cmd (mavros 가 없으므로 z 를 램프로 적분해 위치로 준다). */
  void pubLandSetpoint(double vz);
  /* AGL = odom z - 이륙 시점 지면 z. 기체 z 는 PX4 가 하방 ToF 와 융합한 값이라
     이 축은 신뢰할 수 있다. */
  bool landAgl(double &agl) const;
  /* MAV_CMD_COMPONENT_ARM_DISARM(400) + param2=21196(force magic). */
  bool forceDisarm();
  void mavrosExtendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr &msg);
  int callExplorationPlanner();
  int callGoalPlanner();
  void transitState(EXPL_STATE new_state, string pos_call, bool red = false);
  void battaryCallback(const sensor_msgs::BatteryStateConstPtr &msg);
  bool goalServiceCallback(epic_planner::GoalService::Request& req,
                          epic_planner::GoalService::Response& res);
  /* ROS functions */
  void FSMCallback(const ros::TimerEvent &e);
  // void PlannerDebugFSMCallback(const ros::TimerEvent &e);
  void safetyCallback(const ros::TimerEvent &e);
  void updateTopoAndGlobalPath();
  void globalPathUpdateCallback(const ros::TimerEvent &e);
  void triggerCallback(const nav_msgs::PathConstPtr &msg);
  void avoidFlagCallback(const std_msgs::Int16ConstPtr &msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
  void stopTraj();
  // TAKEOFF_HOVER 종료 공통 경로. yaw_rotate_init_enable_ 이면 YAW_ROTATE_INIT 를
  // 거치고, 아니면 예전처럼 곧바로 탐사로 간다.
  void leaveTakeoffHover(const std::string &why);
  // 탐사 시작 직전 공통 처리 (누적 이동거리 리셋 후 PLAN_TRAJ_EXP).
  void startExplorationFromHover(const std::string &why);
  void pubHoverCmd();  // stream the hover setpoint on /position_cmd during TAKEOFF_HOVER

  // void goal_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void visualize();
  void pubState();

public:
  FastExplorationFSM(/* args */) {}

  ~FastExplorationFSM() {}

  void init(ros::NodeHandle &nh, FastExplorationManager::Ptr &explorer);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace fast_planner
