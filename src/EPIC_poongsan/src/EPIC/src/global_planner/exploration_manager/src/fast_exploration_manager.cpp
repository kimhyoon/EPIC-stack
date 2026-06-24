/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2024-02-25 15:00:51
 * @LastEditTime: 2024-03-12 22:15:11
 * @Description:
 * @
 * @Copyright (c) 2024 by ning-zelin, All Rights Reserved.
 */

#include <boost/lexical_cast.hpp>
#include <epic_planner/expl_data.h>
#include <epic_planner/fast_exploration_manager.h>
#include <fstream>
#include <iostream>
#include <lkh_tsp_solver/lkh_interface.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <plan_manage/planner_manager.h>
#include <std_msgs/Float32.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <visualization_msgs/Marker.h>
using namespace std;
using namespace Eigen;

namespace fast_planner {
// SECTION interfaces for setup and query

FastExplorationManager::FastExplorationManager() {}

FastExplorationManager::~FastExplorationManager() {}

void FastExplorationManager::initialize(
    ros::NodeHandle &nh, FrontierManager::Ptr frt_manager,
    FastPlannerManager::Ptr planner_manager) {

  frontier_manager_ptr_ = frt_manager;
  planner_manager_ = planner_manager;

  ed_.reset(new ExplorationData);
  ep_.reset(new ExplorationParam);
  ed_->next_goal_node_ = make_shared<TopoNode>();

  ep_->a_avg_ = tan(planner_manager_->gcopter_config_->maxTiltAngle) *
                planner_manager_->gcopter_config_->gravAcc;
  ep_->v_max_ = planner_manager_->gcopter_config_->maxVelMag;
  ep_->yaw_v_max_ = planner_manager_->gcopter_config_->yaw_max_vel;
  nh.param("exploration/tsp_dir", ep_->tsp_dir_, string("null"));
  nh.getParam("viewpoint_param/global_viewpoint_num",
              ep_->global_viewpoint_num_);
  nh.getParam("view_graph", ep_->view_graph_);
  nh.getParam("viewpoint_param/local_viewpoint_num", ep_->local_viewpoint_num_);
  nh.getParam("global_planning/w_vdir", ep_->w_vdir_);
  nh.getParam("global_planning/w_yawdir", ep_->w_yawdir_);
  nh.param("fsm/max_segment_length", ep_->max_segment_length_, 3.0);
  Eigen::Vector3d origin, size;
  ofstream par_file(ep_->tsp_dir_ + "/single.par");
  par_file << "PROBLEM_FILE = " << ep_->tsp_dir_ << "/single.tsp\n";
  par_file << "GAIN23 = NO\n";
  par_file << "MOVE_TYPE = 2\n";
  par_file << "OUTPUT_TOUR_FILE =" << ep_->tsp_dir_ << "/single.txt\n";
  par_file << "RUNS = 10\n";
  ros::Duration(1.0).sleep();
  
  // Initialize timing publishers
  insert_viewpoint_cost_pub_ = nh.advertise<std_msgs::Float32>("/global_planning/insert_viewpoint_cost", 10);
  calculate_tsp_cost_pub_ = nh.advertise<std_msgs::Float32>("/global_planning/calculate_tsp_cost", 10);
  lkh_solver_cost_pub_ = nh.advertise<std_msgs::Float32>("/global_planning/lkh_solver_cost", 10);
  topo_graph_search_cost_pub_ = nh.advertise<std_msgs::Float32>("/local_planning/topo_graph_search_cost", 10);
}

void FastExplorationManager::goalCallback(
    const geometry_msgs::PoseStampedConstPtr &msg) {
  // 提取四元数
  double roll, pitch;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg->pose.orientation, quat);

  // 将四元数转换为Euler角
  tf::Matrix3x3(quat).getRPY(roll, pitch, goal_yaw);
}

double FastExplorationManager::getPathCost(TopoNode::Ptr &n1,
                                           Eigen::Vector3d v1, float &yaw1,
                                           TopoNode::Ptr &n2, float &yaw2) {
  auto estimateCost = [&](TopoNode::Ptr &n1, Eigen::Vector3d v1, float &yaw1,
                          TopoNode::Ptr &n2, float &yaw2, int res,
                          vector<Eigen::Vector3f> &path) -> double {
    double len_cost, yaw_cost, dir_cost;
    len_cost = yaw_cost = dir_cost = 0.0;
    if (res == BubbleAstar::NO_PATH)
      return 2e3 + (n1->center_ - n2->center_)
                       .norm(); // 使用一个大的时间值表示无法到达
    if (res == BubbleAstar::START_FAIL || res == BubbleAstar::END_FAIL)
      return 2e3 +
             (n1->center_ - n2->center_).norm(); // 同上，用于不同的错误情况

    len_cost = 0.0;
    for (int i = 0; i < path.size() - 1; ++i)
      len_cost += ((path[i + 1] - path[i]).norm() +
                   0.5 * fabs(path[i + 1].z() - path[i].z()));
    len_cost /= (ep_->v_max_ / 2.0);

    // if (v1.norm() > 1e-3) {
    //   Eigen::Vector3f dir = n2->center_ - n1->center_;
    //   dir.normalize();
    //   Eigen::Vector3f v_dir = v1.normalized().cast<float>();
    //   float yaw1 = atan2(dir.y(), dir.x());
    //   float yaw2 = atan2(v_dir.y(), v_dir.x());
    //   float diff = yaw1 - yaw2;
    //   while (diff > M_PI)
    //     diff -= 2.0 * M_PI;
    //   while (diff < -M_PI)
    //     diff += 2.0 * M_PI;
    //   dir_cost = ep_->w_vdir_ * (fabs(diff) /
    //   planner_manager_->gcopter_config_->yaw_max_vel);
    // }

    // if (path.size() >= 2) {
    //   planner_manager_->calculateTimelb(path, yaw1, yaw2, yaw_cost);
    //   yaw_cost *= ep_->w_yawdir_;
    // }

    return len_cost + dir_cost;
    // return len_cost + dir_cost;
  };
  vector<Eigen::Vector3f> path;
  
  // Measure topo search time
  ros::Time topo_search_start = ros::Time::now();
  int res = planner_manager_->fast_searcher_->topoSearch(n1, n2, 1e-2, path);
  ros::Time topo_search_end = ros::Time::now();
  double topo_search_time = (topo_search_end - topo_search_start).toSec() * 1000.0;
  
  // Publish topo graph search cost
  std_msgs::Float32 topo_search_msg;
  topo_search_msg.data = topo_search_time;
  topo_graph_search_cost_pub_.publish(topo_search_msg);
  
  return estimateCost(n1, v1, yaw1, n2, yaw2, res, path);
}

double FastExplorationManager::getPathCostWithoutTopo(TopoNode::Ptr &n1,
                                                      Eigen::Vector3d v1,
                                                      float &yaw1,
                                                      TopoNode::Ptr &n2,
                                                      float &yaw2) {
  vector<Eigen::Vector3f> path;
  int res = planner_manager_->parallel_path_finder_->search(
      n1->center_, n2->center_, path, 1.0, false);
  if (res != ParallelBubbleAstar::REACH_END)
    return 2e3;
  double cost;
  planner_manager_->parallel_path_finder_->calculatePathCost(path, cost);
  return cost;
}

int FastExplorationManager::planGlobalPath(const Eigen::Vector3d &pos,
                                           const Eigen::Vector3d &vel) {
  bool bm_without_topo = false;
  auto estimiateVdirCost = [&](const TopoNode::Ptr &n1,
                               const Eigen::Vector3d &v1,
                               const TopoNode::Ptr &n2) -> double {
    Eigen::Vector3f dir = n2->center_ - n1->center_;
    dir.normalize();
    Eigen::Vector3f v_dir = v1.normalized().cast<float>();
    float yaw1 = atan2(dir.y(), dir.x());
    float yaw2 = atan2(v_dir.y(), v_dir.x());
    float diff = yaw1 - yaw2;
    while (diff > M_PI)
      diff -= 2.0 * M_PI;
    while (diff < -M_PI)
      diff += 2.0 * M_PI;
    return ep_->w_vdir_ *
           (fabs(diff) / planner_manager_->gcopter_config_->yaw_max_vel);
  };
  ros::Time start = ros::Time::now();
  vector<TopoNode::Ptr> viewpoints;
  frontier_manager_ptr_->generateTSPViewpoints(
      planner_manager_->topo_graph_->odom_node_->center_, viewpoints);

  if (viewpoints.empty()) {
    planner_manager_->graph_visualizer_->vizTour({}, VizColor::RED, "global");
    return NO_FRONTIER;
  }

  ros::Time t1 = ros::Time::now();
  planner_manager_->topo_graph_->insertNodes(viewpoints, false);
  updateGoalNode();
  ros::Time t2 = ros::Time::now();
  // cout << "insert viewpoint to graph time: " << (t2 - t1).toSec() * 1000
  //      << " ms" << endl;
  
  // Publish insert viewpoint cost
  std_msgs::Float32 timing_msg;
  timing_msg.data = (t2 - t1).toSec() * 1000;
  insert_viewpoint_cost_pub_.publish(timing_msg);
  float curr_yaw = (float)planner_manager_->local_data_.curr_yaw_;
  vector<double> distance_odom2vp(viewpoints.size(), 0);
  vector<double> distance_lastgoal2vp(viewpoints.size(), 0);
  double dis2last_goal = 5e3;
  if (planner_manager_->lidar_map_interface_->getDisToOcc(
          ed_->next_goal_node_->center_) >
      planner_manager_->parallel_path_finder_->safe_distance_ + 0.1) {
    dis2last_goal = getPathCost(planner_manager_->topo_graph_->odom_node_,
                                Eigen::Vector3d::Zero(), curr_yaw,
                                ed_->next_goal_node_, curr_yaw);
  }
  static double last_frame_value = dis2last_goal;
  bool last_goal_reachable = dis2last_goal < 2e3;
  // last_goal_reachable = false;

  if (last_goal_reachable && (dis2last_goal < 1.5 * last_frame_value)) {
    last_frame_value = dis2last_goal;
  } else {
    last_goal_reachable = false;
  }

  ros::Time t_start_cvp_1 = ros::Time::now();
  omp_set_num_threads(4);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (int i = 0; i < viewpoints.size(); ++i) {
    if (last_goal_reachable) {
      distance_lastgoal2vp[i] =
          getPathCost(ed_->next_goal_node_, Eigen::Vector3d::Zero(),
                      viewpoints[i]->yaw_, viewpoints[i], viewpoints[i]->yaw_);
      distance_odom2vp[i] =
          getPathCost(planner_manager_->topo_graph_->odom_node_, vel, curr_yaw,
                      viewpoints[i], viewpoints[i]->yaw_);

    } else {
      distance_lastgoal2vp[i] =
          getPathCost(planner_manager_->topo_graph_->odom_node_, vel, curr_yaw,
                      viewpoints[i], viewpoints[i]->yaw_);
      distance_odom2vp[i] = distance_lastgoal2vp[i];
    }
  }
  ros::Time t_end_cvp_1 = ros::Time::now();
  if (bm_without_topo) {
    omp_set_num_threads(4);
    // clang-format off
    #pragma omp parallel for
    // clang-format on
    for (int i = 0; i < viewpoints.size(); ++i) {
      if (last_goal_reachable) {
        distance_lastgoal2vp[i] = getPathCostWithoutTopo(
            ed_->next_goal_node_, Eigen::Vector3d::Zero(), viewpoints[i]->yaw_,
            viewpoints[i], viewpoints[i]->yaw_);
        distance_odom2vp[i] = getPathCostWithoutTopo(
            planner_manager_->topo_graph_->odom_node_, vel, curr_yaw,
            viewpoints[i], viewpoints[i]->yaw_);

      } else {
        distance_lastgoal2vp[i] = getPathCostWithoutTopo(
            planner_manager_->topo_graph_->odom_node_, vel, curr_yaw,
            viewpoints[i], viewpoints[i]->yaw_);
        distance_odom2vp[i] = distance_lastgoal2vp[i];
      }
    }
    ros::Time t_end_cvp_2 = ros::Time::now();
    double cost_mat_with_topo = (t_end_cvp_1 - t_start_cvp_1).toSec() * 1000;
    double cost_mat_without_topo = (t_end_cvp_2 - t_end_cvp_1).toSec() * 1000;
    cout << "cost mat topo: " << cost_mat_with_topo << "ms" << endl;
    cout << "cost mat point cloud: " << cost_mat_without_topo << "ms" << endl;
  }

  vector<TopoNode::Ptr> viewpoint_reachable;
  vector<double> viewpoint_reachable_distance, viewpoint_reachable_distance2;
  for (int i = 0; i < distance_lastgoal2vp.size(); ++i) {
    if (distance_odom2vp[i] > 2e3)
      continue;
    if (last_goal_reachable) {
      viewpoint_reachable_distance.emplace_back(distance_lastgoal2vp[i]);

    } else {
      viewpoint_reachable_distance.emplace_back(distance_odom2vp[i]);
    }
    viewpoint_reachable_distance2.emplace_back(distance_odom2vp[i]);
    viewpoint_reachable.emplace_back(viewpoints[i]);
  }

  if (viewpoint_reachable.empty()) {
    planner_manager_->topo_graph_->removeNodes(viewpoints);
    planner_manager_->graph_visualizer_->vizTour({}, VizColor::RED, "global");
    return NO_FRONTIER;
  }

  if (viewpoint_reachable.size() == 1) {
    ed_->global_tour_.clear();

    ed_->global_tour_.emplace_back(pos.cast<float>());

    ed_->global_tour_.emplace_back(viewpoint_reachable.front()->center_);

    planner_manager_->local_data_.end_yaw_ = viewpoint_reachable.front()->yaw_;
    planner_manager_->topo_graph_->removeNodes(viewpoints);
    return SUCCEED;
  }

  int dim = viewpoint_reachable.size() + 1;
  Eigen::MatrixXd mat;
  mat.resize(dim, dim);
  mat.setZero();
  for (int i = 1; i < dim; ++i) {
    mat(0, i) = viewpoint_reachable_distance[i - 1];
  }

  omp_set_num_threads(4);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (int i = 1; i < dim; i++) {
    for (int j = i + 1; j < dim; j++) {
      mat(i, j) = mat(j, i) = getPathCost(
          viewpoint_reachable[i - 1], Eigen::Vector3d(0, 0, 0),
          viewpoint_reachable[i - 1]->yaw_, viewpoint_reachable[j - 1],
          viewpoint_reachable[j - 1]->yaw_);
    }
  }
  // trick 往远走
  for (int i = 1; i < dim; ++i) {
    mat(i, 0) = 2e3 - viewpoint_reachable_distance2[i - 1] * 0.2;
  }
  for (int i = 0; i < dim; ++i) {
    for (int j = 1; j < dim; ++j) {
      for (int k = 1; k < dim; ++k) {
        if (mat(i, j) > mat(i, k) + mat(k, j)) {
          mat(i, j) = mat(i, k) + mat(k, j) + 1e-2;
        }
      }
    }
  }
  vector<int> indices;
  indices.reserve(dim);
  ros::Time start_tsp = ros::Time::now();
  // cout << "calculate tsp cost matrix cost " << (start_tsp - t2).toSec() * 1000
  //      << "ms" << endl;
  
  // Publish calculate tsp cost
  timing_msg.data = (start_tsp - t2).toSec() * 1000;
  calculate_tsp_cost_pub_.publish(timing_msg);
  solveLHK(mat, indices);
  ros::Time end_tsp = ros::Time::now();
  // cout << "lkh solver cost: " << (end_tsp - start_tsp).toSec() * 1000 << "ms"
  //      << endl;
  
  // Publish lkh solver cost
  timing_msg.data = (end_tsp - start_tsp).toSec() * 1000;
  lkh_solver_cost_pub_.publish(timing_msg);
  // if ((end_tsp - start_tsp).toSec() * 1000 > 100)
  //   exit(0);
  ed_->global_tour_.clear();

  for (auto &i : indices) {
    if (i == 0) {
      ed_->global_tour_.push_back(
          planner_manager_->topo_graph_->odom_node_->center_);
    } else {
      ed_->global_tour_.emplace_back(viewpoint_reachable[i - 1]->center_);
    }
  }
  if (!last_goal_reachable)
    last_frame_value = viewpoint_reachable_distance[indices[1]];

  ros::Time end = ros::Time::now();
  planner_manager_->topo_graph_->removeNodes(viewpoints);

  // Visualize the tour
  planner_manager_->graph_visualizer_->vizTour(ed_->global_tour_, VizColor::BLUE, "global");

  planner_manager_->local_data_.end_yaw_ =
      viewpoint_reachable[indices[1] - 1]->yaw_;
  updateGoalNode();
  return SUCCEED;
}

int FastExplorationManager::planGoalPath(const Eigen::Vector3d &goal_pos, double goal_yaw) {
  ros::Time start = ros::Time::now();

  ROS_INFO("\033[34m[Goal Planning] Start planning to goal: (%.2f, %.2f, %.2f), yaw: %.2f\033[0m",
           goal_pos.x(), goal_pos.y(), goal_pos.z(), goal_yaw);

  // Step 1: Create temporary goal node
  TopoNode::Ptr goal_node = std::make_shared<TopoNode>();
  goal_node->center_ = goal_pos.cast<float>();
  goal_node->yaw_ = goal_yaw;

  // Step 2: Generate and insert frontier viewpoints as stepping stones
  // This helps when goal is in/near unknown regions - frontiers provide intermediate nodes
  // closer to the goal than distant topology skeleton nodes
  vector<TopoNode::Ptr> viewpoints;
  frontier_manager_ptr_->generateTSPViewpoints(
      planner_manager_->topo_graph_->odom_node_->center_, viewpoints);

  if (!viewpoints.empty()) {
    planner_manager_->topo_graph_->insertNodes(viewpoints, false);
    ROS_INFO("\033[36m[Goal Planning] Inserted %lu frontier viewpoints as stepping stones\033[0m",
             viewpoints.size());
  }

  // Step 3: Get current state
  Eigen::Vector3f start_pos = planner_manager_->topo_graph_->odom_node_->center_;
  TopoNode::Ptr start_node = planner_manager_->topo_graph_->odom_node_;

  // Step 4: Connect goal node to topology graph - ONLY verified connections (REACH_END)
  // No optimistic connections to avoid paths through known walls
  vector<TopoNode::Ptr> pre_nbrs;
  planner_manager_->topo_graph_->getPreNbrs(goal_node, pre_nbrs);

  // Try both nearby skeleton nodes AND frontier viewpoints for connections
  vector<TopoNode::Ptr> connection_candidates = pre_nbrs;
  connection_candidates.insert(connection_candidates.end(), viewpoints.begin(), viewpoints.end());

  ROS_INFO("[Goal Planning] Trying to connect goal to %d nodes (%d neighbors + %d frontiers)",
           (int)connection_candidates.size(), (int)pre_nbrs.size(), (int)viewpoints.size());

  int num_connected = 0;
  for (auto& nbr : connection_candidates) {
    vector<Eigen::Vector3f> path;
    Eigen::Vector3f goal_pos_f = goal_pos.cast<float>();
    Eigen::Vector3f nbr_center_f = nbr->center_;

    // Try collision-free path verification with ParallelBubbleAstar
    int res = planner_manager_->parallel_path_finder_->search(
        goal_pos_f, nbr_center_f, path, 1e-3);

    double cost;

    if (res == ParallelBubbleAstar::REACH_END) {
      // ONLY accept verified collision-free connections
      cost = 0.0;
      for (size_t i = 0; i < path.size() - 1; ++i) {
        cost += (path[i] - path[i + 1]).norm();
      }
      cost /= (ep_->v_max_ / 2.0);  // Time cost
      ROS_INFO("\033[32m[Goal Planning] Verified connection to node at (%.2f, %.2f, %.2f), cost: %.2f\033[0m",
               nbr->center_.x(), nbr->center_.y(), nbr->center_.z(), cost);

      // Add bidirectional edge
      goal_node->neighbors_.insert(nbr);
      goal_node->weight_[nbr] = cost;
      goal_node->paths_[nbr] = path;

      nbr->neighbors_.insert(goal_node);
      nbr->weight_[goal_node] = cost;
      nbr->paths_[goal_node] = path;

      num_connected++;

    } else {
      // Reject ALL non-verified connections (NO_PATH, TIME_OUT, START_FAIL, END_FAIL)
      // This eliminates optimistic connections through potentially known walls
      const char* reason;
      if (res == ParallelBubbleAstar::NO_PATH) reason = "known obstacle";
      else if (res == ParallelBubbleAstar::START_FAIL) reason = "start in collision";
      else if (res == ParallelBubbleAstar::END_FAIL) reason = "end in collision";
      else if (res == ParallelBubbleAstar::TIME_OUT) reason = "timeout (rejected - no optimistic connections)";
      else reason = "unknown error";

      // Only log at debug level to reduce spam
      ROS_DEBUG("[Goal Planning] Skipping connection to (%.2f, %.2f, %.2f) - %s",
                nbr->center_.x(), nbr->center_.y(), nbr->center_.z(), reason);
    }
  }

  ROS_INFO("[Goal Planning] Connected goal to %d topology nodes (verified only)", num_connected);

  // FALLBACK: If goal is not reachable, navigate to closest frontier instead
  if (num_connected == 0) {
    ROS_WARN("\033[33m[Goal Planning] Goal not reachable! Fallback: Navigate to closest frontier\033[0m");

    // Find closest frontier to goal (before removing viewpoints!)
    if (viewpoints.empty()) {
      ROS_ERROR("[Goal Planning] No frontiers available for fallback!");
      return NO_FRONTIER;
    }

    Eigen::Vector3f goal_pos_f = goal_pos.cast<float>();
    double min_dist = std::numeric_limits<double>::max();
    TopoNode::Ptr closest_frontier = nullptr;
    for (auto& vp : viewpoints) {
      double dist = (vp->center_ - goal_pos_f).norm();
      if (dist < min_dist) {
        min_dist = dist;
        closest_frontier = vp;
      }
    }

    ROS_INFO("\033[33m[Goal Planning] Redirecting to closest frontier at (%.2f, %.2f, %.2f), dist: %.2fm from goal\033[0m",
             closest_frontier->center_.x(), closest_frontier->center_.y(), closest_frontier->center_.z(), min_dist);

    // Now plan a path to the closest frontier instead of the goal
    // Use the closest frontier as the new goal
    goal_node->center_ = closest_frontier->center_;
    goal_node->yaw_ = closest_frontier->yaw_;

    // Try connecting to this frontier position
    vector<TopoNode::Ptr> frontier_candidates;
    planner_manager_->topo_graph_->getPreNbrs(goal_node, frontier_candidates);

    // Also try connecting to other nearby skeleton nodes
    for (auto& nbr : frontier_candidates) {
      vector<Eigen::Vector3f> path;
      int res = planner_manager_->parallel_path_finder_->search(
          goal_node->center_, nbr->center_, path, 1e-3);

      if (res == ParallelBubbleAstar::REACH_END) {
        double cost = 0.0;
        for (size_t i = 0; i < path.size() - 1; ++i) {
          cost += (path[i] - path[i + 1]).norm();
        }
        cost /= (ep_->v_max_ / 2.0);

        goal_node->neighbors_.insert(nbr);
        goal_node->weight_[nbr] = cost;
        goal_node->paths_[nbr] = path;

        nbr->neighbors_.insert(goal_node);
        nbr->weight_[goal_node] = cost;
        nbr->paths_[goal_node] = path;

        num_connected++;
        ROS_INFO("\033[32m[Goal Planning] Connected frontier fallback to node at (%.2f, %.2f, %.2f)\033[0m",
                 nbr->center_.x(), nbr->center_.y(), nbr->center_.z());
      }
    }

    if (num_connected == 0) {
      ROS_ERROR("[Goal Planning] Even closest frontier is unreachable!");
      // Cleanup
      if (!viewpoints.empty()) {
        planner_manager_->topo_graph_->removeNodes(viewpoints);
      }
      return FAIL;
    }

    ROS_INFO("[Goal Planning] Frontier fallback connected to %d nodes", num_connected);
  }

  // Step 4: A* search on topology graph
  vector<TopoNode::Ptr> topo_path;
  ros::Time search_start = ros::Time::now();
  bool search_success = planner_manager_->topo_graph_->graphSearch(
      start_node, goal_node, topo_path, 0.1);  // 100ms timeout
  ros::Time search_end = ros::Time::now();

  std_msgs::Float32 timing_msg;
  timing_msg.data = (search_end - search_start).toSec() * 1000;
  topo_graph_search_cost_pub_.publish(timing_msg);

  if (!search_success || topo_path.empty()) {
    ROS_WARN("[Goal Planning] A* search failed on topology graph!");

    // Cleanup: remove goal node connections
    for (auto& nbr : connection_candidates) {
      nbr->neighbors_.erase(goal_node);
      nbr->weight_.erase(goal_node);
      nbr->paths_.erase(goal_node);
    }

    // Cleanup: remove frontier viewpoints
    if (!viewpoints.empty()) {
      planner_manager_->topo_graph_->removeNodes(viewpoints);
    }

    return FAIL;
  }

  ROS_INFO("\033[32m[Goal Planning] A* found path with %lu nodes\033[0m", topo_path.size());

  // Step 5: Build global_tour_ from topology path
  ed_->global_tour_.clear();

  ed_->global_tour_.push_back(start_pos);

  for (size_t i = 1; i < topo_path.size(); ++i) {
    ed_->global_tour_.push_back(topo_path[i]->center_);
  }

  // Add final goal position
  ed_->global_tour_.push_back(goal_pos.cast<float>());

  // Set end yaw
  planner_manager_->local_data_.end_yaw_ = goal_yaw;

  // Step 6: Simplify global_tour using greedy visibility checks
  simplifyGlobalTour();

  // Step 7: Cleanup - remove goal node connections and frontier viewpoints
  for (auto& nbr : connection_candidates) {
    nbr->neighbors_.erase(goal_node);
    nbr->weight_.erase(goal_node);
    nbr->paths_.erase(goal_node);
  }

  // Remove frontier viewpoints (stepping stones no longer needed)
  if (!viewpoints.empty()) {
    planner_manager_->topo_graph_->removeNodes(viewpoints);
    ROS_INFO("[Goal Planning] Removed %lu frontier stepping stones", viewpoints.size());
  }

  // Visualize the tour
  planner_manager_->graph_visualizer_->vizTour(ed_->global_tour_, VizColor::BLUE, "goal");

  ros::Time end = ros::Time::now();
  ROS_INFO("\033[32m[Goal Planning] Total planning time: %.2f ms\033[0m",
           (end - start).toSec() * 1000);

  return SUCCEED;
}

void FastExplorationManager::simplifyGlobalTour() {
  if (ed_->global_tour_.size() <= 2) {
    // No simplification needed for start-goal only
    return;
  }

  ros::Time t_start = ros::Time::now();
  int original_size = ed_->global_tour_.size();

  vector<Eigen::Vector3f> simplified_tour;

  simplified_tour.push_back(ed_->global_tour_[0]);  // Always keep start

  const double MAX_SEGMENT_LENGTH = ep_->max_segment_length_;
  int collision_checks = 0;

  int anchor_idx = 0;

  while (anchor_idx < ed_->global_tour_.size() - 1) {
    int furthest_visible = anchor_idx + 1;  // At least move to next node

    // Greedy search: find furthest visible node from anchor
    for (int test_idx = anchor_idx + 2; test_idx < ed_->global_tour_.size(); ++test_idx) {
      Eigen::Vector3f anchor_pos = ed_->global_tour_[anchor_idx];
      Eigen::Vector3f test_pos = ed_->global_tour_[test_idx];

      // Check segment length constraint first
      double segment_length = (test_pos - anchor_pos).norm();
      if (segment_length > MAX_SEGMENT_LENGTH) {
        break;  // Too far, stop searching
      }

      // Test visibility using parallel_path_finder
      vector<Eigen::Vector3f> path;
      collision_checks++;
      int res = planner_manager_->parallel_path_finder_->search(
          anchor_pos, test_pos, path, 1e-3);

      if (res == ParallelBubbleAstar::REACH_END) {
        // Visible! Update furthest
        furthest_visible = test_idx;
        // Continue testing further nodes (don't break)
      } else {
        // Hit obstacle, stop searching (greedy: can't skip further if blocked here)
        break;
      }
    }

    // Add the furthest visible node
    if (furthest_visible < ed_->global_tour_.size() - 1) {
      simplified_tour.push_back(ed_->global_tour_[furthest_visible]);
    }

    // Move anchor to furthest visible
    anchor_idx = furthest_visible;
  }

  // Always add final goal
  simplified_tour.push_back(ed_->global_tour_.back());

  // Update global_tour_
  ed_->global_tour_ = simplified_tour;

  ros::Time t_end = ros::Time::now();
  ROS_INFO("\033[33m[Path Simplification] Reduced path from %d to %lu nodes, %d collision checks, %.2f ms\033[0m",
           original_size, simplified_tour.size(), collision_checks,
           (t_end - t_start).toSec() * 1000.0);

  // Print segment lengths for debugging
  for (size_t i = 1; i < simplified_tour.size(); ++i) {
    double seg_length = (simplified_tour[i] - simplified_tour[i-1]).norm();
    ROS_INFO("  Segment %lu: %.2f m", i, seg_length);
  }
}

void FastExplorationManager::solveLHK(Eigen::MatrixXd &cost_mat,
                                      vector<int> &indices) {
  // Solve linear homogeneous kernel
  // Write params and cost matrix to problem file
  int dimension = cost_mat.rows();
  if (dimension < 3)
    return;
  ofstream prob_file(ep_->tsp_dir_ + "/single.tsp");
  // Problem specification part, follow the format of TSPLIB

  string prob_spec =
      "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
      "\nEDGE_WEIGHT_TYPE : "
      "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";

  // string prob_spec = "NAME : single\nTYPE : TSP\nDIMENSION : " +
  // to_string(dimension) +
  //     "\nEDGE_WEIGHT_TYPE : "
  //     "EXPLICIT\nEDGE_WEIGHT_FORMAT : LOWER_ROW\nEDGE_WEIGHT_SECTION\n";

  prob_file << prob_spec;
  // prob_file << "TYPE : TSP\n";
  // prob_file << "EDGE_WEIGHT_FORMAT : LOWER_ROW\n";
  // Problem data part
  const int scale = 100;

  // Use Asymmetric TSP
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = cost_mat(i, j) * scale;
      prob_file << int_cost << " ";
    }
    prob_file << "\n";
  }

  prob_file << "EOF";
  prob_file.close();

  // Call LKH TSP solver
  solveTSPLKH((ep_->tsp_dir_ + "/single.par").c_str());

  // Read optimal tour from the tour section of result file
  ifstream res_file(ep_->tsp_dir_ + "/single.txt");
  string res;
  while (getline(res_file, res)) {
    // Go to tour section
    if (res.compare("TOUR_SECTION") == 0)
      break;
  }

  // Read path for ATSP formulation
  while (getline(res_file, res)) {

    // Read indices of frontiers in optimal tour
    int id = stoi(res);
    if (id == -1)
      break;
    indices.push_back(id - 1); // Idx of solver-2 == Idx of frontier
  }

  res_file.close();
}

void FastExplorationManager::updateGoalNode() {
  if (ed_->global_tour_.empty())
    return;
  Eigen::Vector3f goal = ed_->global_tour_[1];

  struct PairPtrHash {
    std::size_t
    operator()(const std::pair<TopoNode::Ptr, TopoNode::Ptr> &p) const {
      return std::hash<TopoNode::Ptr>()(p.first) ^
             std::hash<TopoNode::Ptr>()(p.second);
    }
  };

  Eigen::Vector3i idx;
  planner_manager_->topo_graph_->getIndex(goal, idx);
  vector<TopoNode::Ptr> pre_nbrs;
  for (int i = -1; i <= 1; i++)
    for (int j = -1; j <= 1; j++)
      for (int k = -1; k <= 1; k++) {
        Eigen::Vector3i tmp_idx = idx;
        tmp_idx(0) = idx(0) + i;
        tmp_idx(1) = idx(1) + j;
        tmp_idx(2) = idx(2) + k;
        auto region = planner_manager_->topo_graph_->getRegionNode(tmp_idx);
        if (region) {
          for (auto &topo : region->topo_nodes_) {
            if (topo == ed_->next_goal_node_)
              continue;
            pre_nbrs.emplace_back(topo);
          }
        }
      }
  std::unordered_map<std::pair<TopoNode::Ptr, TopoNode::Ptr>,
                     vector<Eigen::Vector3f>, PairPtrHash>
      edge2insert;
  mutex edge2insert_mtx;
  omp_set_num_threads(4);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (auto &nbr : pre_nbrs) {
    vector<Eigen::Vector3f> path;
    int res = planner_manager_->topo_graph_->parallel_bubble_astar_->search(
        goal, nbr->center_, path, 1e-3);
    if (res == ParallelBubbleAstar::REACH_END &&
        planner_manager_->topo_graph_->parallel_bubble_astar_
            ->collisionCheck_shortenPath(path)) {
      edge2insert_mtx.lock();
      edge2insert.insert({std::make_pair(ed_->next_goal_node_, nbr), path});
      edge2insert_mtx.unlock();
    }
  }
  // 更新goal节点
  ed_->next_goal_node_->center_ = goal;
  ed_->next_goal_node_->is_viewpoint_ = true;
  if (edge2insert.size() > 0) {
    planner_manager_->topo_graph_->removeNode(ed_->next_goal_node_);
    for (auto &edge : edge2insert) {
      ed_->next_goal_node_->neighbors_.insert(edge.first.second);
      ed_->next_goal_node_->paths_.insert({edge.first.second, edge.second});
      double cost;
      planner_manager_->topo_graph_->parallel_bubble_astar_->calculatePathCost(
          edge.second, cost);
      ed_->next_goal_node_->weight_[edge.first.second] = cost;
      auto nbr = edge.first.second;
      nbr->neighbors_.insert(ed_->next_goal_node_);
      nbr->weight_[ed_->next_goal_node_] = cost;
      vector<Eigen::Vector3f> path = edge.second;
      std::reverse(path.begin(), path.end());
      nbr->paths_[ed_->next_goal_node_] = path;
    }
  }
}
} // namespace fast_planner