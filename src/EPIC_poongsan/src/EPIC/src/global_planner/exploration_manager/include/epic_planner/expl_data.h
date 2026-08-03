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

  // --- EARLY_FINISH probe (EFP): FSM -> planGlobalPath 경계 ---------------
  // FSM(FastExplorationFSM)이 매 전역계획 직전에 갱신하고, planGlobalPath 가
  // 읽어서 TSP 후보 목록에 EFP 를 한 개 더 얹는다. 그래서 EFP 가 살아있는 동안엔
  // viewpoints 가 절대 비지 않고 -> NO_FRONTIER 가 안 나고 -> FINISH/RTH 가
  // 자동으로 봉쇄된다 (별도 게이트 코드 없음).
  //
  // efp_node_ 는 **이미 topo graph 안에 있는 실제 노드**다. 따라서
  // insertNodes/removeNodes 대상에서 반드시 제외해야 한다 (removeNodes 는 이웃과
  // region 에서 노드를 통째로 뜯어내므로 그래프가 망가진다).
  bool efp_active_ = false;
  TopoNode::Ptr efp_node_;
  Eigen::Vector3f efp_pos_ = Eigen::Vector3f::Zero();
  float efp_yaw_ = 0.0f;
  // odom -> EFP 그래프 거리[m] (FSM 의 Dijkstra 결과). getPathCost 내부
  // topoSearch 가 예산 초과로 거짓 NO_PATH 를 낼 때의 대체 비용으로 쓴다.
  double efp_graph_dist_ = 0.0;
  // planGlobalPath 가 채운다: 이번 주기 global_tour_ 안에서의 EFP 인덱스(없으면 -1).
  // tour 색 분리(파랑 tour / 자주색 EFP leg)에 쓴다.
  int efp_tour_index_ = -1;
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
  // [feature: topo-timeout] topo graph A* 예산 [s]. 셋 다 같은 graphSearch 를
  // 호출하는데 원래는 호출부마다 다른 값이 박혀 있었다 (10ms / 100ms, 그리고
  // 뷰포인트 도달성은 0.3ms — ViewpointManager/reachability_search_timeout).
  // 예산이 모자라면 그래프가 연결돼 있어도 "경로 없음"과 구분되지 않는다.
  double topo_cost_search_timeout_;  // getPathCost: TSP 비용 행렬용
  double goal_search_timeout_;       // planGoalPath: RTH/서비스 목표 경로용
};

} // namespace fast_planner
