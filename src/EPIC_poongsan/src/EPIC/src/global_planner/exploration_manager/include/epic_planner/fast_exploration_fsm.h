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
#include <string>
#include <thread>
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
enum EXPL_STATE { INIT, WAIT_TRIGGER, PLAN_TRAJ_EXP, PLAN_TRAJ_RTH, CAUTION, EXEC_TRAJ, FINISH, LAND, TAKEOFF_HOVER, LANDED };

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
  void logGlobalPlanEvent(int res, double t_ms); // GLOBAL 이벤트 공통 발행
  bool verbose_console_ = false;      // true 면 기존 타이밍 cout/INFO 유지
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

  /* helper functions */
  bool explorationReallyFinished();          // false during startup warmup, true once frontiers seen / timeout
  void mavrosStateCallback(const mavros_msgs::State::ConstPtr &msg);
  void pubHoldCmd(const Eigen::Vector3d &p, double yaw);  // stream a fixed-pose hover setpoint on /position_cmd
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
