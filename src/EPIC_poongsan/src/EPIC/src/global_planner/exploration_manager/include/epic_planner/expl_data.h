#pragma once

#include <Eigen/Eigen>
#include <pointcloud_topo/graph.h>
#include <traj_utils/PolyTraj.h>
#include <vector>
using Eigen::Vector3d;
using std::vector;
using namespace std;

namespace fast_planner {
struct FSMData {
  // FSM data
  bool trigger_, have_odom_, static_state_, emergency_replan_,
      use_bubble_a_star_, half_resolution;
  vector<string> state_str_;
  int bb_astar_fail_cnt_, fast_search_fial_cnt_;
  double bb_astar_time_out, fast_search_time_out;
  Eigen::Vector3f odom_pos_, odom_vel_; // odometry state
  Eigen::Quaterniond odom_orient_;
  float odom_yaw_;

  Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_; // start state
  vector<Eigen::Vector3d> start_poss;
  traj_utils::PolyTraj newest_traj_;
  traj_utils::PolyTraj newest_yaw_traj_;
};

struct FSMParam {
  double replan_thresh_;
  double replan_time_after_traj_start_;
  double replan_time_before_traj_end_;
  double replan_time_; // second
  double emergency_replan_control_error;
  double bubble_a_star_resolution;
  // takeoff & hover-before-explore (real flight): on trigger, climb to
  // takeoff_height_ and hold until odom confirms the drone is stable near it,
  // then auto-start exploration. Set takeoff_height_ <= 0 to disable (old behaviour).
  double takeoff_height_;       // target hover altitude [m] in odom-frame z
  double takeoff_reach_tol_;    // |z - target| below this counts as "reached" [m]
  double takeoff_settle_vel_;   // odom speed below this counts as "stable" [m/s]
  double takeoff_settle_time_;  // must stay reached+stable this long before explore [s]
  double takeoff_timeout_;      // give up waiting after this and explore anyway [s]
};

struct ExplorationData {
  vector<vector<Vector3d>> frontiers_;
  vector<vector<Vector3d>> dead_frontiers_;
  vector<pair<Vector3d, Vector3d>> frontier_boxes_;
  vector<Vector3d> points_;
  vector<Vector3d> averages_;
  vector<Vector3d> views_;
  vector<double> yaws_;
  vector<TopoNode::Ptr> local_tour_;
  vector<Eigen::Vector3f> global_tour_;
  TopoNode::Ptr first_root_, second_root_;

  Eigen::Vector3f next_goal_;
  TopoNode::Ptr next_goal_node_;
  vector<Eigen::Vector3f> path_next_goal_;

  // viewpoint planning
  // vector<Vector4d> views_;
  vector<Vector3d> views_vis1_, views_vis2_;
  vector<Vector3d> centers_, scales_;
  Eigen::Vector3f tsp_end_node_;

  // --- debug diagnostics (rviz HUD + logging), populated by planGlobalPath() ---
  int diag_num_clusters_ = 0;            // frontier 클러스터 총 개수 (모든 박스)
  int diag_num_clusters_reachable_ = 0;  // 도달 가능+활성 클러스터 (초록 박스)
  int diag_num_viewpoints_ = 0;          // 이번 계획에서 생성된 뷰포인트 수
  int diag_num_reachable_vp_ = 0;        // 토포 그래프로 도달 가능한 뷰포인트 수
  std::string diag_reason_ = "init";     // 마지막 계획 결과 / 실패 사유 (상세)
  // 짧은 결과 분류 (이벤트 dedup 키): OK / NO_VIEWPOINTS / NO_REACHABLE_VP /
  // RTH_PATH_OK / RTH_FALLBACK_FRONTIER / RTH_NO_FRONTIER / RTH_ASTAR_FAIL / RTH_FAIL
  std::string diag_result_ = "init";
};

struct ExplorationParam {
  // params
  int local_viewpoint_num_, global_viewpoint_num_;
  int viewpoint_connection_num_;
  double a_avg_, v_max_, yaw_v_max_, viewpoint_gian_lambda_;
  double w_vdir_, w_yawdir_;
  bool view_graph_;
  string tsp_dir_; // resource dir of tsp solver
  double max_segment_length_; // maximum segment length for path simplification
};

} // namespace fast_planner
