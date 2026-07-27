/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2023-12-21 21:31:51
 * @LastEditTime: 2024-03-10 11:39:33
 * @Description:
 * @
 * @Copyright (c) 2023 by ning-zelin, All Rights Reserved.
 */

#ifndef _EXPLORATION_MANAGER_H_
#define _EXPLORATION_MANAGER_H_

#include <Eigen/Eigen>
#include <frontier_manager/frontier_manager.h>
#include <memory>
#include <omp.h>
#include <opencv2/opencv.hpp>
#include <pointcloud_topo/graph.h>
#include <plan_manage/planner_manager.h>
#include <ros/ros.h>
#include <vector>
using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
class EDTEnvironment;
class SDFMap;
class FastPlannerManager;
struct ExplorationParam;
struct ExplorationData;

enum EXPL_RESULT { NO_FRONTIER, FAIL, START_FAIL, SUCCEED };

class FastExplorationManager {
public:
  typedef shared_ptr<FastExplorationManager> Ptr;
  FastExplorationManager();
  ~FastExplorationManager();
  shared_ptr<ExplorationData> ed_;
  shared_ptr<ExplorationParam> ep_;
  ros::Timer frontier_timer_;
  FrontierManager::Ptr frontier_manager_ptr_;
  double goal_yaw;

  shared_ptr<FastPlannerManager> planner_manager_;
  // ViewpointForest::Ptr vps_forest_;
  
  // Publishers for timing data
  ros::Publisher insert_viewpoint_cost_pub_, calculate_tsp_cost_pub_, lkh_solver_cost_pub_,
                 topo_graph_search_cost_pub_;
  double getPathCost(TopoNode::Ptr &n1, Eigen::Vector3d v1, float &yaw1, TopoNode::Ptr &n2, float &yaw2);
  double getPathCostWithoutTopo(TopoNode::Ptr &n1, Eigen::Vector3d v1, float &yaw1, TopoNode::Ptr &n2, float &yaw2);
  void initialize(ros::NodeHandle &nh, FrontierManager::Ptr frt_manager,
                  FastPlannerManager::Ptr planner_manager);
  int planGlobalPath(const Vector3d &pos, const Vector3d &vel);
  int planGoalPath(const Vector3d &goal_pos, double goal_yaw);
  void simplifyGlobalTour();
  void solveLHK(Eigen::MatrixXd &cost_mat, vector<int> &indices);

  void surfaceFrtCalllback(const ros::TimerEvent &e);
  void goalCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  void updateGoalNode();
  // [EARLY_FINISH] odom 노드에서 토포 그래프를 Dijkstra 로 한 번 훑어 도달
  // 가능한 노드 중 경로 비용이 가장 큰 노드를 돌려준다. 노드마다 graphSearch 를
  // 부르면 O(N) 번의 탐색이 되므로 한 번의 전방향 확장으로 대체한다.
  // 히스토리 odom 노드(이미 지나온 궤적)는 제외한다 — 다시 가봐야 새로 관측될
  // 것이 없어 early-finish 의 목적(새 영역 관측)에 맞지 않는다.
  TopoNode::Ptr findFarthestReachableNode(double &cost_out);
};

} // namespace fast_planner

#endif