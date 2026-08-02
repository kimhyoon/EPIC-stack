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
#include <epic_planner/fast_exploration_manager.h>
#include <iostream>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pointcloud_topo/graph_visualizer.hpp>
#include <quadrotor_msgs/TakeoffLand.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <mavros_msgs/SetMode.h>
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

// NOTE: TAKEOFF_HOVER/LANDED are appended at the END so existing enum indices
// (used as indices into fd_->state_str_) stay unchanged.
// LANDED: LAND(AUTO.LAND) 후 착지+disarm 이 확인된 최종 상태.
//         record_on_goal 이 이 상태를 보고 녹화를 마감한다.
// EARLY_FINISH: 탐사가 너무 짧게 끝났을 때(누적 이동거리 < thresh), 현재 odom
//               node 에서 가장 먼 도달 가능 topology node 까지의 기존 graph path 를
//               global tour 로 설치한 뒤 PLAN_TRAJ_EXP 로 돌아간다.
// YAW_ROTATE_INIT: 이륙 호버 지점을 유지한 채 제자리에서 360도 회전해 주변을 한
//         바퀴 관측한 뒤 탐사를 시작한다. 이륙 직후 맵이 비어 frontier 가 0개로
//         보이는 상황에 대한 사전 예방이다 (EARLY_FINISH 는 같은 문제의 사후 구제).
//         새 enum 도 END 에 추가해 기존 인덱스를 건드리지 않는다.
enum EXPL_STATE { INIT, WAIT_TRIGGER, PLAN_TRAJ_EXP, PLAN_TRAJ_RTH, CAUTION, EXEC_TRAJ, FINISH, LAND, TAKEOFF_HOVER, LANDED, EARLY_FINISH, YAW_ROTATE_INIT };

class FastExplorationFSM {
private:
  /* planning utils */
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<FastExplorationManager> expl_manager_;
  shared_ptr<PlanningVisualization> visualization_;

  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  EXPL_STATE state_;

  bool classic_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, global_path_update_timer_;
  ros::Subscriber trigger_sub_, map_update_sub_, battary_sub_, avoid_flag_sub_;
  ros::Publisher stop_pub_, new_pub_, replan_pub_, poly_traj_pub_, heartbeat_pub_, time_cost_pub_, poly_yaw_traj_pub_, static_pub_, state_pub_,
  land_pub_, rth_metrics_pub_, hover_cmd_pub_;
  ros::Publisher early_finish_state_pub_;
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

  SynchronizerCloudOdom sync_cloud_odom_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
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
     takeoff point -> when within xy tol of home, switch PX4 to AUTO.LAND. */
  bool   auto_rth_land_ = true;              // master enable
  bool   traj_server_owns_finish_cmd_ = false;  // release /position_cmd to traj_server in FINISH
  double finish_hover_duration_ = 3.0;       // [s] hover at last cmd pose before returning
  double rth_land_xy_tol_ = 0.3;             // [m] xy proximity to home that triggers landing
  bool   explore_finished_ = false;          // latched only when EXPLORATION ends (not service-RTH)
  bool   returning_home_ = false;            // auto RTH-then-land in progress (routes RTH->LAND)
  ros::Time finish_hover_start_;             // when the FINISH hover began (0 = not yet)
  Eigen::Vector3d finish_hover_pos_ = Eigen::Vector3d::Zero();  // snapshot of last cmd pose
  double finish_hover_yaw_ = 0.0;
  ros::ServiceClient set_mode_client_;       // /mavros/set_mode (for AUTO.LAND)
  ros::Subscriber    mavros_state_sub_;      // /mavros/state (to confirm AUTO.LAND engaged)
  mavros_msgs::State px4_state_;

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
  double early_finish_dist_thresh_ = 3.0;      // [m] max_displacement_ 가 이 미만이면 조기종료 의심
  int    early_finish_max_retry_ = 1;          // 재시도 횟수 상한 (무한루프 방지 래치)
  // [m] 자동 구제 2단 판정: EFP 후보의 원점거리가 max_displacement_ + 이 값을
  // 넘어야 probe 를 보낸다. 이미 가본 범위 안이면 관측 이득이 없기 때문.
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

  /* helper functions */
  // 탐사 박스(lp_->global_box_*) 안인가. box_num_<=0 이면 제한 없음(true).
  bool insideExplorationBox(const Eigen::Vector3f &p) const;
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
  void mavrosStateCallback(const mavros_msgs::State::ConstPtr &msg);
  // yaw_dot 을 실어 보내면 px4_ctrl_bridge(use_yawrate=true)가 yawrate 로 회전시킨다.
  void pubHoldCmd(const Eigen::Vector3d &p, double yaw, double yaw_dot = 0.0);
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
