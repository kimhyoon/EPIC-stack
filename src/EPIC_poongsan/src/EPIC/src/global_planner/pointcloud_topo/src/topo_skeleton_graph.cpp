/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2023-12-02 21:33:08
 * @LastEditTime: 2024-03-05 12:12:19
 * @Description:
 * @
 * @Copyright (c) 2024 by ning-zelin, All Rights Reserved.
 */

#include "pointcloud_topo/graph.h"

#include <cmath>
#include <limits>

namespace {
enum TopologyNodeUpdateAction {
  NODE_MATCHED = 1,
  NODE_RETAINED_BY_HYSTERESIS = 2,
  NODE_REMOVED_UNSAFE = 3,
  NODE_REMOVED_MISS_EXPIRED = 4,
  NODE_INSERTED = 5,
  NODE_REJECTED_ADMISSION = 6,
};

struct TopologyNodeUpdateEvent {
  int action;
  TopoNode::Ptr node;
  double clearance;
  int missed_before;
  double nearest_new_distance;
  size_t neighbor_count;
};
} // namespace

void debug_exit(const std::string &location) {
  std::cout << "\033[1;31m Terminating process at location: " << location << "\033[0m" << std::endl;
  exit(0);
}

void TopoGraph::init(ros::NodeHandle &nh, LIOInterface::Ptr &lidar_map, ParallelBubbleAstar::Ptr &parallel_bubble_astar) {
  lidar_map_interface_ = lidar_map;

  min_bd = lidar_map_interface_->lp_->global_box_min_boundary_;
  max_bd = lidar_map_interface_->lp_->global_box_max_boundary_;

  parallel_bubble_astar_ = parallel_bubble_astar;
  odom_node_ = make_shared<TopoNode>();
  odom_node_->is_viewpoint_ = true;
  odom_node_->stable_id_ = next_node_id_++;
  // 分区，初始化regions_arr_
  // 10m * 10m * 2m ==> 0.315 * 0.315 * 0.5
  nh = nh;
  nh.param("bubble_topo/min_x", min_x_, 0.0);
  nh.param("bubble_topo/min_y", min_y_, 0.0);
  nh.param("bubble_topo/min_z", min_z_, 0.0);
  nh.param("bubble_topo/init_region_size_x", init_region_size_x_, 0.0);
  nh.param("bubble_topo/init_region_size_y", init_region_size_y_, 0.0);
  nh.param("bubble_topo/init_region_size_z", init_region_size_z_, 0.0);
  nh.param("bubble_topo/frontier_bubble_min_radius", frt_bubble_radius_, 0.5);
  nh.param("bubble_topo/cube_discrete_size", cube_discrete_size, 0.3);
  nh.param("bubble_topo/odom_node_distance", odom_node_distance_, 5.0);
  nh.param("bubble_topo/node_match_tolerance", node_match_tolerance_, 0.05);
  nh.param("bubble_topo/node_miss_hysteresis", node_miss_hysteresis_, 2);
  nh.param("bubble_topo/node_insert_margin", node_insert_margin_, 0.0);
  nh.param("bubble_topo/node_clearance_tie_tolerance",
           node_clearance_tie_tolerance_, 0.0);
  if (node_insert_margin_ < 0.0) {
    ROS_WARN("[TopoGraph] bubble_topo/node_insert_margin must be non-negative; "
             "clamping %.3f to 0.0",
             node_insert_margin_);
    node_insert_margin_ = 0.0;
  }
  if (node_clearance_tie_tolerance_ < 0.0) {
    ROS_WARN("[TopoGraph] bubble_topo/node_clearance_tie_tolerance must be "
             "non-negative; clamping %.3f to 0.0",
             node_clearance_tie_tolerance_);
    node_clearance_tie_tolerance_ = 0.0;
  }
  nh.param("planner_debug/enable", planner_debug_enabled_, false);

  nh.getParam("parallel_astar/update_connection_timeout", update_connection_timeout);
  nh.getParam("parallel_astar/insert_node_timeout", insert_node_timeout);

  nh.getParam("max_update_region_num", max_update_region_num_);
  update_idx_vec_.reserve(100);
  global_path_.reserve(200);
  x_len = std::ceil((max_bd - min_bd).x() / init_region_size_x_);
  y_len = std::ceil((max_bd - min_bd).y() / init_region_size_y_);
  z_len = std::ceil((max_bd - min_bd).z() / init_region_size_z_);
  for (size_t i = 0; i < x_len; i++)
    for (int j = 0; j < y_len; j++)
      for (int k = 0; k < z_len; k++) {
        Eigen::Vector3i idx(i, j, k);
        RegionNode::Ptr region_node = std::make_shared<RegionNode>(idx);
        Eigen::Vector3f hb, lb;
        index2boundary(idx, lb, hb);
        if (!hasOverlapWithBox(lb, hb))
          continue;
        reg_map_idx2ptr_[idx] = region_node;
      }
  // 建立一个octree作为球形覆盖的check-point
  check_pts_.clear();
  for (float x = 0; x < init_region_size_x_; x += cube_discrete_size) {
    for (float y = 0; y < init_region_size_y_; y += cube_discrete_size) {
      for (float z = 0; z < init_region_size_z_; z += cube_discrete_size) {
        check_pts_.emplace_back(x, y, z);
      }
    }
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr check_pts_pc(new pcl::PointCloud<pcl::PointXYZ>);
  check_pts_pc->points = check_pts_;
  check_pts_octree_.setResolution(cube_discrete_size);
  check_pts_octree_.setInputCloud(check_pts_pc);
  check_pts_octree_.addPointsFromInputCloud();
  
  // Initialize timing publisher
  bubble_astar_search_cost_pub_ = nh.advertise<std_msgs::Float32>("/local_planning/bubble_astar_search_cost", 10);
  if (planner_debug_enabled_) {
    topo_edge_debug_pub_ = nh.advertise<std_msgs::Float64MultiArray>(
        "/debug/topo_edge_checks", 10);
    topo_edge_update_debug_pub_ = nh.advertise<std_msgs::Float64MultiArray>(
        "/debug/topo_edge_updates", 10);
    topology_node_updates_pub_ = nh.advertise<std_msgs::Float64MultiArray>(
        "/debug/topology_node_updates", 10);
    topology_stability_nodes_pub_ = nh.advertise<visualization_msgs::Marker>(
        "/debug/topology_stability_nodes", 1, true);
    topology_failed_edges_pub_ = nh.advertise<visualization_msgs::Marker>(
        "/debug/topology_failed_edges", 1, true);
    ROS_WARN("[PlannerDebug] topology edge evidence publisher enabled");
  }
}

BubbleNode::BubbleNode(double radius, Eigen::Vector3f center) {
  radius_ = radius;
  center_ = center;
}

RegionNode::RegionNode(Eigen::Vector3i region_idx) {
  region_idx_ = region_idx;
  his_odom_id_ = -1;
}

RegionNode::Ptr TopoGraph::getRegionNode(const Eigen::Vector3i &region_idx_) {
  if (reg_map_idx2ptr_.find(region_idx_) == reg_map_idx2ptr_.end()) {
    return nullptr;
  }
  return reg_map_idx2ptr_[region_idx_];
}

void TopoGraph::getIndex(const Eigen::Vector3f &point, Eigen::Vector3i &region_idx_) {
  region_idx_.x() = int((point[0] - min_bd[0]) / init_region_size_x_);
  region_idx_.y() = int((point[1] - min_bd[1]) / init_region_size_y_);
  region_idx_.z() = int((point[2] - min_bd[2]) / init_region_size_z_);
}

bool TopoGraph::index2boundary(const Eigen::Vector3i &region_idx_, Eigen::Vector3f &low_bd, Eigen::Vector3f &high_bd) {
  low_bd = Eigen::Vector3f(min_bd[0] + region_idx_.x() * init_region_size_x_, min_bd[1] + region_idx_.y() * init_region_size_y_,
                           min_bd[2] + region_idx_.z() * init_region_size_z_);
  high_bd = low_bd + Eigen::Vector3f(init_region_size_x_, init_region_size_y_, init_region_size_z_);
  return true;
}


void BubbleUnionSet::init(const std::vector<BubbleNode::Ptr> &bubbles_) {
  parent.clear();
  rank.clear();
  clusters.clear();
  bubbles.clear();
  topo_map.clear();
  bubbles = bubbles_;
  rank.reserve(bubbles.size());
  parent.reserve(bubbles.size());
  clusters.reserve(bubbles.size());
  parent.clear();
  rank.clear();
  for (auto &b : bubbles) {
    parent[b] = b;
    rank[b] = 0;
  }
}

BubbleNode::Ptr BubbleUnionSet::find(BubbleNode::Ptr b) {
  if (parent[b] != b) {
    parent[b] = find(parent[b]);
  }
  return parent[b];
}

void BubbleUnionSet::merge(BubbleNode::Ptr b1, BubbleNode::Ptr b2) {
  b1 = find(b1);
  b2 = find(b2);
  if (rank[b1] > rank[b2]) {
    parent[b2] = b1;
  } else {
    parent[b1] = b2;
    if (rank[b1] == rank[b2]) {
      rank[b2]++;
    }
  }
}

void BubbleUnionSet::getClusters() {
  clusters.clear();
  for (auto &b : parent) {
    if (b.second == b.first) {
      clusters.push_back(b.first);
      topo_map[b.first] = TopoNode::Ptr(new TopoNode);
    }
  }
  for (auto &b : bubbles) {
    auto topo_ptr = topo_map[find(b)];
    topo_ptr->bubbles_.push_back(b);
  }
}

bool TopoGraph::graphSearch(const TopoNode::Ptr &start_node, const TopoNode::Ptr &end_node, std::vector<TopoNode::Ptr> &path, double time_out,
                            bool kino, std::unordered_set<pair<TopoNode::Ptr, TopoNode::Ptr>, PairPtrHash> last_path,
                            int *result_code) {
  if (result_code)
    *result_code = ParallelBubbleAstar::NO_PATH;
  ros::Time search_start = ros::Time::now();
  
  path.clear();
  std::unordered_map<TopoNode::Ptr, float> g_score, f_score;
  std::unordered_map<TopoNode::Ptr, TopoNode::Ptr> parent_map;
  std::unordered_set<TopoNode::Ptr> close_set, open_set_set_;
  float tie_breaker_ = 1.0 + 1.0 / 1000;
  std::priority_queue<std::pair<float, TopoNode::Ptr>, std::vector<std::pair<float, TopoNode::Ptr>>, std::greater<std::pair<float, TopoNode::Ptr>>>
  open_set;
  auto getHeuristic = [&](const TopoNode::Ptr &n) -> float { return tie_breaker_ * (n->center_ - end_node->center_).norm(); };
  auto backtrack = [&]() {
    TopoNode::Ptr cur_node = end_node;
    path.push_back(cur_node);
    while (parent_map.find(cur_node) != parent_map.end()) {
      cur_node = parent_map[cur_node];
      path.push_back(cur_node);
    }
    std::reverse(path.begin(), path.end());
  };
  auto cur_node = start_node;
  std::unordered_map<TopoNode::Ptr, Eigen::Vector3f> node_vel;
  g_score[cur_node] = 0.0;
  f_score[cur_node] = getHeuristic(cur_node);
  open_set.push({f_score[cur_node], cur_node});
  open_set_set_.insert(cur_node);
  const auto t1 = ros::Time::now();
  while (!open_set.empty()) {
    cur_node = open_set.top().second;
    open_set_set_.erase(cur_node);
    open_set.pop();
    close_set.insert(cur_node);
    if (cur_node == end_node) {
      backtrack();
      if (result_code)
        *result_code = ParallelBubbleAstar::REACH_END;
      return true;
    }
    if ((ros::Time::now() - t1).toSec() > time_out) {
      // ROS_ERROR("topo a* timeout");
      if (result_code)
        *result_code = ParallelBubbleAstar::TIME_OUT;
      return false;
    }
    for (auto &neighbor : cur_node->neighbors_) {
      // if (!neighbor->reachable_)
      //   continue;
      if (close_set.find(neighbor) != close_set.end())
        continue;

      float tentative_g_score;
      if (kino) {
        if (last_path.find({cur_node, neighbor}) != last_path.end()) {
          // tentative_g_score = g_score[cur_node] + 1e-3 * cur_node->weight_[neighbor];
          tentative_g_score = g_score[cur_node] + 0 * cur_node->weight_[neighbor];
        } else
          tentative_g_score = g_score[cur_node] + cur_node->weight_[neighbor];
      } else {
        tentative_g_score = g_score[cur_node] + cur_node->weight_[neighbor];
      }
      if (open_set_set_.find(neighbor) == open_set_set_.end() || tentative_g_score < g_score[neighbor]) {
        parent_map[neighbor] = cur_node;
        g_score[neighbor] = tentative_g_score;
        f_score[neighbor] = g_score[neighbor] + getHeuristic(neighbor);
        open_set.push({f_score[neighbor], neighbor});
        open_set_set_.insert(neighbor);
      } else
        continue;
    }
  }
  
  // Publish timing information
  double search_time = (ros::Time::now() - search_start).toSec() * 1000.0;
  // Note: This would need access to a visualizer instance to publish
  // For now, we'll add this timing measurement but the actual publishing
  // will be done in the calling function
  
  return false;
}

void TopoGraph::cauculateMemoryConsumption() {
  size_t graph_cost = 0;
  size_t graph_cost2 = 0;

  double node_size = 0;
  double nbr_size = 0;
  double ur_nbr_size = 0;
  for (auto &[_, region] : reg_map_idx2ptr_) {
    size_t single_cost = 0;
    size_t single_cost2 = 0;
    if (region->topo_nodes_.empty())
      continue;
    for (auto &topo : region->topo_nodes_) {
      node_size++;
      nbr_size += topo->neighbors_.size();
      ur_nbr_size += topo->unreachable_nbrs_.size();
      if (topo->neighbors_.size() != topo->paths_.size()) {
        ROS_ERROR("memory error 644");
        exit(1);
      }
      if (topo->neighbors_.size() != topo->weight_.size()) {
        ROS_ERROR("memory error 648");
        exit(1);
      }

      single_cost += sizeof(bool);                                                       // is_viewpoint_
      single_cost += sizeof(float);                                                      // yaw_
      single_cost += sizeof(Eigen::Vector3f);                                            // center
      single_cost += (sizeof(TopoNode::Ptr) + 1) * topo->neighbors_.size();              // neighbors_
      single_cost += (sizeof(TopoNode::Ptr) + 2) * topo->unreachable_nbrs_.size();       // unreachable_nbrs_
      single_cost += (sizeof(float) + sizeof(TopoNode::Ptr) + 1) * topo->weight_.size(); // weight_
      single_cost2 = single_cost;
      single_cost2 -= (sizeof(TopoNode::Ptr) + 1) * topo->neighbors_.size();
      for (auto &nei : topo->neighbors_) {
        single_cost += sizeof(Eigen::Vector3f) * topo->paths_[nei].size(); // paths_
        single_cost2 += sizeof(Eigen::Vector3f) * topo->paths_[nei].size() / 2.0;
      }
    }
    single_cost += (sizeof(Eigen::Vector3i) + sizeof(RegionNode::Ptr) + 1); // region_node key-value pair
    graph_cost += single_cost;
    graph_cost2 += single_cost2;
  }
  for (auto &topo : history_odom_nodes_) {
    size_t single_cost = 0;
    size_t single_cost2 = 0;
    single_cost += sizeof(bool);                                                       // is_viewpoint_
    single_cost += sizeof(float);                                                      // yaw_
    single_cost += sizeof(Eigen::Vector3f);                                            // center
    single_cost += (sizeof(TopoNode::Ptr) + 1) * topo->neighbors_.size();              // neighbors_
    single_cost += (sizeof(TopoNode::Ptr) + 2) * topo->unreachable_nbrs_.size();       // unreachable_nbrs_
    single_cost += (sizeof(float) + sizeof(TopoNode::Ptr) + 1) * topo->weight_.size(); // weight_
    single_cost2 = single_cost;
    single_cost2 -= (sizeof(TopoNode::Ptr) + 1) * topo->neighbors_.size();
    for (auto &nei : topo->neighbors_) {
      single_cost += sizeof(Eigen::Vector3f) * topo->paths_[nei].size(); // paths_
      single_cost2 += sizeof(Eigen::Vector3f) * topo->paths_[nei].size() / 2.0;
    }

    single_cost += (sizeof(Eigen::Vector3i) + sizeof(RegionNode::Ptr) + 1); // region_node key-value pair
    graph_cost += single_cost;
    graph_cost2 += single_cost2;
  }

}

int TopoGraph::getBoxId(const Eigen::Vector3f &pt) {
  auto inbox = [&](const Eigen::Vector3f &pt, const Eigen::Vector3f &min, const Eigen::Vector3f &max) -> bool {
    for (size_t i = 0; i < 3; i++) {
      if (pt(i) < min(i) || pt(i) > max(i))
        return false;
    }
    return true;
  };

  for (size_t i = 0; i < lidar_map_interface_->lp_->box_num_; i++) {
    Eigen::Vector3f min_ = lidar_map_interface_->lp_->global_box_min_boundary_vec_[i];
    Eigen::Vector3f max_ = lidar_map_interface_->lp_->global_box_max_boundary_vec_[i];
    if (inbox(pt, min_, max_))
      return i;
  }
  return -1;
}

void BubbleUnionSet::unionSetCluster(const vector<BubbleNode::Ptr> &bubbles, vector<TopoNode::Ptr> &topos, Eigen::Vector3f &center) {
  auto is_bubble_connected = [&](BubbleNode::Ptr b1, BubbleNode::Ptr b2) -> bool {
    double center_distance = (b1->center_ - b2->center_).norm();
    return center_distance < (b1->radius_ + b2->radius_) - 0.5;
  };
  init(bubbles);
  for (size_t i = 0; i < bubbles.size(); i++)
    for (int j = i + 1; j < bubbles.size(); j++)
      if (is_bubble_connected(bubbles[i], bubbles[j]))
        merge(bubbles[i], bubbles[j]);
  getClusters();
  // getTopoNodes(topos, center);
  for (auto &tpair : topo_map) {
    auto node = tpair.second;
    if (node->bubbles_.empty())
      continue;
    BubbleNode::Ptr max_radius_bubble = node->bubbles_[0];
    BubbleNode::Ptr max_admissible_bubble;
    for (auto &b : node->bubbles_) {
      if (b->radius_ > max_radius_bubble->radius_)
        max_radius_bubble = b;
      if (b->radius_ >= min_topobubble_radius_ &&
          (!max_admissible_bubble ||
           b->radius_ > max_admissible_bubble->radius_))
        max_admissible_bubble = b;
    }

    BubbleNode::Ptr selected_bubble = max_radius_bubble;
    if (max_admissible_bubble) {
      const double equivalent_clearance_min =
          max_admissible_bubble->radius_ - clearance_tie_tolerance_;
      double selected_distance =
          std::numeric_limits<double>::infinity();
      for (auto &b : node->bubbles_) {
        if (b->radius_ < min_topobubble_radius_ ||
            b->radius_ < equivalent_clearance_min)
          continue;

        const double distance = (b->center_ - center).norm();
        if (distance < selected_distance) {
          selected_bubble = b;
          selected_distance = distance;
        }
      }
    }

    node->center_ = selected_bubble->center_;
    node->is_viewpoint_ = false;
    topos.push_back(node);
    vector<BubbleNode::Ptr>().swap(node->bubbles_);
  }
}

void TopoGraph::overlap(vector<TopoNode::Ptr> &set1, vector<TopoNode::Ptr> &set2, vector<TopoNode::Ptr> &overlap) {
  HashMap map;
  for (auto &node : set1) {
    Eigen::Vector3i idx;
    posToIndex(node->center_, idx);
    map[idx] = node;
  }
  overlap.clear();
  for (auto &node : set2) {
    Eigen::Vector3i idx;
    posToIndex(node->center_, idx);
    if (map.find(idx) != map.end()) {
      overlap.push_back(node);
    }
  }
}

void TopoGraph::setdiff(vector<TopoNode::Ptr> &set1, vector<TopoNode::Ptr> &set2, vector<TopoNode::Ptr> &set_1diff2) {
  HashMap map;
  for (auto &node : set2) {
    Eigen::Vector3i idx;
    posToIndex(node->center_, idx);
    map[idx] = node;
  }
  set_1diff2.clear();
  for (auto &node : set1) {
    Eigen::Vector3i idx;
    posToIndex(node->center_, idx);
    if (map.find(idx) == map.end())
      set_1diff2.push_back(node);
  }
}

void TopoGraph::removeNodes(vector<TopoNode::Ptr> &nodes) {

  // region_set
  for (auto &node : nodes) {
    if (node == nullptr)
      continue;
    Eigen::Vector3i region_idx;
    getIndex(node->center_, region_idx);
    auto region_node = getRegionNode(region_idx);
    ROS_ASSERT(region_node != nullptr);
    // if (region_node == nullptr) {
    //   continue;
    //   debug_exit("TopoGraph::removeNodes :region_node == nullptr ");
    // }
    region_node->topo_nodes_.erase(node);
  }

  // nbrs
  for (auto &node : nodes) {
    if (node == nullptr)
      continue;
    for (auto &nbr : node->neighbors_) {
      // if (nbr->is_history_odom_node_)
      //   continue;
      nbr->neighbors_.erase(node);
      nbr->paths_.erase(node);
      nbr->weight_.erase(node);
      nbr->unreachable_nbrs_.erase(node);
      nbr->edge_failures_.erase(node);
    }
    for (auto &entry : node->edge_failures_)
      entry.first->edge_failures_.erase(node);
    node->unreachable_nbrs_.clear();
    node->edge_failures_.clear();
    node->neighbors_.clear();
    node->weight_.clear();
    node->paths_.clear();
  }
}

bool TopoGraph::updateRemainedConnections(vector<TopoNode::Ptr> &nodes) {
  auto retryDelay = [](int result) {
    if (result == ParallelBubbleAstar::TIME_OUT)
      return 0.2;
    if (result == ParallelBubbleAstar::START_FAIL ||
        result == ParallelBubbleAstar::END_FAIL)
      return 0.5;
    return 1.0;
  };

  auto checkNbr = [&](PtrPair::iter_elem &elem) {
    vector<Eigen::Vector3f> path = elem.p1->paths_[elem.p2];
    if (parallel_bubble_astar_->collisionCheck_shortenPath(path)) {
      elem.insert = true;
      elem.result = ParallelBubbleAstar::REACH_END;
      elem.path = path;
      return;
    }

    path.clear();
    int result = searchPathWithBoundary(elem.p1->center_, elem.p2->center_,
                                        update_connection_timeout, path);
    if (result == ParallelBubbleAstar::REACH_END &&
        parallel_bubble_astar_->collisionCheck_shortenPath(path)) {
      elem.insert = true;
      elem.result = ParallelBubbleAstar::REACH_END;
      elem.path = path;
    } else {
      elem.insert = false;
      elem.result = result == ParallelBubbleAstar::REACH_END
                        ? EDGE_COLLISION
                        : result;
    }
  };

  auto testPreNbr = [&](PtrPair::iter_elem &elem) {
    vector<Eigen::Vector3f> path;
    int result = searchPathWithBoundary(elem.p1->center_, elem.p2->center_,
                                        update_connection_timeout, path);
    if (result == ParallelBubbleAstar::REACH_END &&
        parallel_bubble_astar_->collisionCheck_shortenPath(path)) {
      elem.insert = true;
      elem.result = ParallelBubbleAstar::REACH_END;
      elem.path = path;
    } else {
      elem.insert = false;
      elem.result = result == ParallelBubbleAstar::REACH_END
                        ? EDGE_COLLISION
                        : result;
    }
  };

  PtrPair edge2test, edge2check;
  const ros::Time now = ros::Time::now();
  for (auto &node : nodes) {
    vector<TopoNode::Ptr> pre_nbrs;
    getPreNbrs(node, pre_nbrs);
    unordered_set<TopoNode::Ptr> pre_nbrs_set(pre_nbrs.begin(),
                                              pre_nbrs.end());
    for (auto &nbr : node->neighbors_) {
      if (!nbr->is_history_odom_node_)
        pre_nbrs_set.insert(nbr);
    }

    unordered_map<TopoNode::Ptr, uint8_t> unreachable_nbrs_tmp;
    unordered_map<TopoNode::Ptr, TopoNode::EdgeFailureState>
        edge_failures_tmp;
    for (auto &entry : node->unreachable_nbrs_) {
      if (pre_nbrs_set.count(entry.first) && entry.first != odom_node_)
        unreachable_nbrs_tmp.insert(entry);
    }
    for (auto &entry : node->edge_failures_) {
      if (pre_nbrs_set.count(entry.first) && entry.first != odom_node_)
        edge_failures_tmp.insert(entry);
    }
    node->unreachable_nbrs_.swap(unreachable_nbrs_tmp);
    node->edge_failures_.swap(edge_failures_tmp);

    for (auto &pre_nbr : pre_nbrs_set) {
      const bool connected = node->neighbors_.count(pre_nbr) > 0;
      if (!connected && node->edge_failures_.count(pre_nbr)) {
        const auto &failure = node->edge_failures_.at(pre_nbr);
        if ((now - failure.last_failure_time).toSec() <
            retryDelay(failure.last_result))
          continue;
      }
      if (connected)
        edge2check.insert(node, pre_nbr);
      else
        edge2test.insert(node, pre_nbr);
    }
  }

  edge2test.flatten();
  edge2check.flatten();
  for (auto &elem : edge2test.flatten_data)
    elem.was_connected = false;
  for (auto &elem : edge2check.flatten_data)
    elem.was_connected = true;

  omp_set_num_threads(6);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (auto &elem : edge2test.flatten_data)
    testPreNbr(elem);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (auto &elem : edge2check.flatten_data)
    checkNbr(elem);

  edge2test.flatten_data.insert(edge2test.flatten_data.end(),
                                edge2check.flatten_data.begin(),
                                edge2check.flatten_data.end());

  std_msgs::Float64MultiArray debug_msg;
  const bool publish_debug =
      planner_debug_enabled_ &&
      topo_edge_update_debug_pub_.getNumSubscribers() > 0;
  if (publish_debug) {
    // Schema v1: [version, batch_seq, pair_count,
    //             success, result, was_connected, start_xyz, end_xyz,
    //             consecutive_failures, ...].
    debug_msg.data.reserve(3 + edge2test.flatten_data.size() * 10);
    debug_msg.data.push_back(1.0);
    debug_msg.data.push_back(static_cast<double>(++topo_debug_batch_seq_));
    debug_msg.data.push_back(
        static_cast<double>(edge2test.flatten_data.size()));
  }

  bool graph_changed = false;
  for (auto &elem : edge2test.flatten_data) {
    auto node1 = elem.p1;
    auto node2 = elem.p2;
    uint16_t failure_count = 0;
    if (elem.insert) {
      node1->paths_[node2] = elem.path;
      vector<Eigen::Vector3f> reverse_path = elem.path;
      std::reverse(reverse_path.begin(), reverse_path.end());
      node2->paths_[node1] = reverse_path;
      double cost;
      parallel_bubble_astar_->calculatePathCost(elem.path, cost);
      node1->unreachable_nbrs_.erase(node2);
      node2->unreachable_nbrs_.erase(node1);
      node1->edge_failures_.erase(node2);
      node2->edge_failures_.erase(node1);
      node1->neighbors_.insert(node2);
      node2->neighbors_.insert(node1);
      node1->weight_[node2] = cost;
      node2->weight_[node1] = cost;
      if (!elem.was_connected)
        graph_changed = true;
    } else {
      if (elem.was_connected)
        graph_changed = true;
      node1->neighbors_.erase(node2);
      node2->neighbors_.erase(node1);
      node1->weight_.erase(node2);
      node2->weight_.erase(node1);
      node1->paths_.erase(node2);
      node2->paths_.erase(node1);

      auto recordFailure = [&](TopoNode::Ptr &from, TopoNode::Ptr &to) {
        auto &failure = from->edge_failures_[to];
        if (failure.consecutive_failures <
            std::numeric_limits<uint16_t>::max())
          failure.consecutive_failures++;
        failure.last_result = elem.result;
        failure.last_failure_time = now;
        auto &legacy_count = from->unreachable_nbrs_[to];
        if (legacy_count < std::numeric_limits<uint8_t>::max())
          legacy_count++;
        return failure.consecutive_failures;
      };
      failure_count = recordFailure(node1, node2);
      recordFailure(node2, node1);
    }

    if (publish_debug) {
      debug_msg.data.push_back(elem.insert ? 1.0 : 0.0);
      debug_msg.data.push_back(static_cast<double>(elem.result));
      debug_msg.data.push_back(elem.was_connected ? 1.0 : 0.0);
      for (int axis = 0; axis < 3; ++axis)
        debug_msg.data.push_back(node1->center_[axis]);
      for (int axis = 0; axis < 3; ++axis)
        debug_msg.data.push_back(node2->center_[axis]);
      debug_msg.data.push_back(static_cast<double>(failure_count));
    }
  }
  if (publish_debug)
    topo_edge_update_debug_pub_.publish(debug_msg);
  return graph_changed;
}

void TopoGraph::getPreNbrs(TopoNode::Ptr &node, vector<TopoNode::Ptr> &nbrs) {
  Eigen::Vector3i idx, odom_idx;
  getIndex(node->center_, idx);
  nbrs.clear();
  // getIndex(odom_node_->center_, odom_idx);
  // Eigen::Vector3i diff = (idx - odom_idx).cwiseAbs();
  // if (diff.maxCoeff() <= 1)
  //   nbrs.push_back(odom_node_);
  vector<Eigen::Vector3i> steps1{Eigen::Vector3i(0, 0, 0),  Eigen::Vector3i(1, 0, 0), Eigen::Vector3i(-1, 0, 0), Eigen::Vector3i(0, 1, 0),
                                 Eigen::Vector3i(0, -1, 0), Eigen::Vector3i(0, 0, 1), Eigen::Vector3i(0, 0, -1)};
  vector<Eigen::Vector3i> steps2{Eigen::Vector3i(1, 1, 0), Eigen::Vector3i(1, -1, 0), Eigen::Vector3i(-1, 1, 0), Eigen::Vector3i(-1, -1, 0),
                                 Eigen::Vector3i(1, 0, 1), Eigen::Vector3i(1, 0, -1), Eigen::Vector3i(-1, 0, 1), Eigen::Vector3i(-1, 0, -1),
                                 Eigen::Vector3i(0, 1, 1), Eigen::Vector3i(0, 1, -1), Eigen::Vector3i(0, -1, 1), Eigen::Vector3i(0, -1, -1)};

  // for (int i = 0; i < steps1.size() + steps2.size(); i++) {
  //   if (i >= steps1.size() && nbrs.size() > 4)
  //     break;
  for (int i = 0; i < steps1.size() ; i++) {
    Eigen::Vector3i step = i < steps1.size() ? steps1[i] : steps2[i - steps1.size()];
    Eigen::Vector3i nbr_idx = idx + step;
    auto nbr_region_node = getRegionNode(nbr_idx);
    if (nbr_region_node == nullptr)
      continue;
    for (auto &nbr_topo_node : nbr_region_node->topo_nodes_) {
      if (nbr_topo_node == nullptr) {
        cout << "wtf 970" << endl;
        continue;
      }
      if (nbr_topo_node == node)
        continue;
      // if (nbr_topo_node->is_viewpoint_ && node->is_viewpoint_)
      //   continue;
      nbrs.push_back(nbr_topo_node);
    }
  }
}

// void TopoGraph::getPreNbrs(TopoNode::Ptr &node, vector<TopoNode::Ptr> &nbrs) {
//   Eigen::Vector3i idx, odom_idx;
//   getIndex(node->center_, idx);
//   if (!node->is_viewpoint_) {
//     for (int i = 0; i < 3; i++) {
//       for (int j = -1; j <= 1; j++) {
//         Eigen::Vector3i idx_tmp = idx;
//         idx_tmp[i] += j;
//         auto nbr_region_node = getRegionNode(idx_tmp);
//         if (nbr_region_node == nullptr)
//           continue;
//         for (auto &nbr_topo_node : nbr_region_node->topo_nodes_) {
//           if (nbr_topo_node == nullptr)
//             continue;
//           if (nbr_topo_node == node)
//             continue;
//           nbrs.push_back(nbr_topo_node);
//         }
//       }
//     }
//   } else {
//     for (int i = -1; i <= 1; i++) {
//       for (int j = -1; j <= 1; j++) {
//         for (int k = -1; k <= 1; k++) {
//           Eigen::Vector3i idx_tmp = idx;
//           idx_tmp[0] += i;
//           idx_tmp[1] += j;
//           idx_tmp[2] += k;
//           auto nbr_region_node = getRegionNode(idx_tmp);
//           if (nbr_region_node == nullptr)
//             continue;
//           for (auto &nbr_topo_node : nbr_region_node->topo_nodes_) {
//             if (nbr_topo_node == nullptr)
//               continue;
//             if (nbr_topo_node == node)
//               continue;
//             nbrs.push_back(nbr_topo_node);
//           }
//         }
//       }
//     }
//   }
// }

void TopoGraph::insertNodes(vector<TopoNode::Ptr> &nodes, bool only_raycast) {
  // insert到region里
  if (nodes.empty())
    return;
  for (auto &node : nodes) {
    if (node == nullptr)
      continue;
    if (!node->is_viewpoint_ && node->stable_id_ == 0)
      node->stable_id_ = next_node_id_++;
    node->connection_attempt_count_ = 0;
    node->last_connection_result_ = ParallelBubbleAstar::NO_PATH;
    Eigen::Vector3i region_idx;
    getIndex(node->center_, region_idx);
    // else
    auto region_node = getRegionNode(region_idx);
    // if (region_node == nullptr) {
    //   continue;
    // }
    ROS_ASSERT(region_node != nullptr);
    region_node->topo_nodes_.insert(node);
  }

  // 找到邻居region和自己region的其他节点

  std::unordered_set<std::pair<TopoNode::Ptr, TopoNode::Ptr>, PairPtrHash> ptr_pair_set;
  vector<pair<TopoNode::Ptr, TopoNode::Ptr>> pair_vector; // 使用vector支持并行运算
  for (auto &node : nodes) {
    vector<TopoNode::Ptr> nbrs;
    getPreNbrs(node, nbrs);
    for (auto &nbr : nbrs) {
      if (ptr_pair_set.find({node, nbr}) == ptr_pair_set.end()) {
        pair_vector.push_back({node, nbr});
        ptr_pair_set.insert({node, nbr});
        ptr_pair_set.insert({nbr, node});
      }
    }
  }

  // 获得节点对的vector
  vector<vector<Eigen::Vector3f>> path_vec; // 初始是start和end两个点, 算完是path+一个zero/one表示成功/失败
  path_vec.resize(pair_vector.size());
  vector<int> result_vec(pair_vector.size(), ParallelBubbleAstar::NO_PATH);

  // 并行A*搜索路径并写入结果
  omp_set_num_threads(6);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (size_t i = 0; i < path_vec.size(); i++) {
    Eigen::Vector3f start = pair_vector[i].first->center_;
    Eigen::Vector3f end = pair_vector[i].second->center_;
    vector<Eigen::Vector3f> path;
    int res;
    if (!only_raycast) {
      res = searchPathWithBoundary(start, end, insert_node_timeout, path);
    } else {
      // Measure bubble astar search time
      ros::Time bubble_search_start = ros::Time::now();
      res = parallel_bubble_astar_->search(start, end, path, insert_node_timeout, false, true);
      ros::Time bubble_search_end = ros::Time::now();
      double bubble_search_time = (bubble_search_end - bubble_search_start).toSec() * 1000.0;
      
      // Publish bubble astar search cost
      std_msgs::Float32 bubble_search_msg;
      bubble_search_msg.data = bubble_search_time;
      bubble_astar_search_cost_pub_.publish(bubble_search_msg);
    }
    if (res != ParallelBubbleAstar::REACH_END)
      path.push_back(Eigen::Vector3f::Zero());
    else if (!only_raycast) {
      bool safe = parallel_bubble_astar_->collisionCheck_shortenPath(path);
      if (safe)
        path.push_back(Eigen::Vector3f::Ones());
      else {
        res = EDGE_COLLISION;
        path.push_back(Eigen::Vector3f::Zero());
      }
    } else {
      path.push_back(Eigen::Vector3f::Ones()); // 1表示安全，0表示危险
    }
    result_vec[i] = res;
    path_vec[i].swap(path);
  }

  // 串行更新节点
  std_msgs::Float64MultiArray debug_msg;
  const bool publish_debug =
      planner_debug_enabled_ && topo_edge_debug_pub_.getNumSubscribers() > 0;
  if (publish_debug) {
    // Schema v2: [version, batch_seq, only_raycast, pair_count,
    //             success, result, start_xyz, end_xyz, path_point_count,
    //             path_cost, min_known_obstacle_distance, ...].
    debug_msg.data.reserve(4 + pair_vector.size() * 11);
    debug_msg.data.push_back(2.0);
    debug_msg.data.push_back(static_cast<double>(++topo_debug_batch_seq_));
    debug_msg.data.push_back(only_raycast ? 1.0 : 0.0);
    debug_msg.data.push_back(static_cast<double>(pair_vector.size()));
  }
  for (size_t i = 0; i < path_vec.size(); i++) {
    const bool success =
        !path_vec[i].empty() && path_vec[i].back().norm() >= 0.5;
    if (publish_debug) {
      debug_msg.data.push_back(success ? 1.0 : 0.0);
      debug_msg.data.push_back(static_cast<double>(result_vec[i]));
      for (int axis = 0; axis < 3; ++axis)
        debug_msg.data.push_back(pair_vector[i].first->center_[axis]);
      for (int axis = 0; axis < 3; ++axis)
        debug_msg.data.push_back(pair_vector[i].second->center_[axis]);
      const size_t path_point_count =
          path_vec[i].empty() ? 0 : path_vec[i].size() - 1;
      debug_msg.data.push_back(static_cast<double>(path_point_count));
    }
    auto node1 = pair_vector[i].first;
    auto node2 = pair_vector[i].second;
    node1->connection_attempt_count_++;
    node2->connection_attempt_count_++;
    if (!success) {
      if (node1->last_connection_result_ != ParallelBubbleAstar::REACH_END)
        node1->last_connection_result_ = result_vec[i];
      if (node2->last_connection_result_ != ParallelBubbleAstar::REACH_END)
        node2->last_connection_result_ = result_vec[i];
      if (publish_debug) {
        debug_msg.data.push_back(-1.0);
        debug_msg.data.push_back(-1.0);
      }
      continue;
    }
    node1->last_connection_result_ = ParallelBubbleAstar::REACH_END;
    node2->last_connection_result_ = ParallelBubbleAstar::REACH_END;
    node1->edge_failures_.erase(node2);
    node2->edge_failures_.erase(node1);
    node1->neighbors_.insert(node2);
    node2->neighbors_.insert(node1);
    path_vec[i].pop_back();
    node1->paths_[node2] = path_vec[i];
    std::reverse(path_vec[i].begin(), path_vec[i].end());
    node2->paths_[node1] = path_vec[i];
    double cost;
    parallel_bubble_astar_->calculatePathCost(path_vec[i], cost);
    node1->weight_[node2] = cost;
    node2->weight_[node1] = cost;
    if (publish_debug) {
      double min_distance = std::numeric_limits<double>::infinity();
      for (const auto &point : path_vec[i]) {
        const Eigen::Vector3d point_d = point.cast<double>();
        const double distance = lidar_map_interface_->getDisToOcc(point_d);
        if (std::isfinite(distance) && distance < min_distance)
          min_distance = distance;
      }
      debug_msg.data.push_back(cost);
      debug_msg.data.push_back(
          std::isfinite(min_distance) ? min_distance : -1.0);
    }
  }
  if (publish_debug)
    topo_edge_debug_pub_.publish(debug_msg);
}

void TopoGraph::getRegionsToUpdate() {
  update_idx_vec_.clear();
  viewpoints_update_region_arr_.clear();
  toponodes_update_region_arr_.clear();
  unordered_set<RegionNode::Ptr> region_set;
  for (auto &pt : lidar_map_interface_->ld_->lidar_cloud_.points) {
    Eigen::Vector3f pt3d = pt.getArray3fMap();
    if ((pt3d - lidar_map_interface_->ld_->lidar_pose_).norm() > lidar_map_interface_->lp_->max_ray_length_)
      pt3d = lidar_map_interface_->ld_->lidar_pose_ + lidar_map_interface_->lp_->max_ray_length_ * (pt3d - lidar_map_interface_->ld_->lidar_pose_) /
                                                   (pt3d - lidar_map_interface_->ld_->lidar_pose_).norm();
    Eigen::Vector3i region_idx;
    getIndex(pt3d, region_idx);
    auto region = getRegionNode(region_idx);
    if (region != nullptr)
      region_set.insert(region);
  }
  for (auto &region : region_set) {
    toponodes_update_region_arr_.push_back(region);
  }
  auto shorten_by_distance_insert_update_arr = [&](vector<RegionNode::Ptr> &arr) {
    std::sort(arr.begin(), arr.end(), [this](const RegionNode::Ptr &region1, const RegionNode::Ptr &region2) {
      Eigen::Vector3f lb1, hb1, lb2, hb2;
      index2boundary(region1->region_idx_, lb1, hb1);
      index2boundary(region2->region_idx_, lb2, hb2);
      Eigen::Vector3f diff1 = ((hb1 + lb1) * 0.5 - lidar_map_interface_->ld_->lidar_pose_);
      Eigen::Vector3f diff2 = ((hb2 + lb2) * 0.5 - lidar_map_interface_->ld_->lidar_pose_);
      double dist1 = diff1.norm();
      double dist2 = diff2.norm();
      return dist1 < dist2;
    });
    arr.resize(std::min((int)arr.size(), max_update_region_num_));

    // 去重
    unordered_set<RegionNode::Ptr> region2update(arr.begin(), arr.end());
    arr = vector<RegionNode::Ptr>(region2update.begin(), region2update.end());
  };
  // shorten_by_distance_insert_update_arr(toponodes_update_region_arr_);

  // 向四周发射射线，超过当前单位球大概一格子的范围
  double step_size = min(init_region_size_x_, init_region_size_y_);
  step_size = min(step_size, init_region_size_z_);
  step_size /= 2.0;
  for (auto &region : toponodes_update_region_arr_) {
    Eigen::Vector3f lb, hb, goal;
    index2boundary(region->region_idx_, lb, hb);
    goal = 0.5 * (lb + hb);
    Eigen::Vector3f dir = goal - lidar_map_interface_->ld_->lidar_pose_;
    int step_num = (int)(dir.norm() / step_size) + 1;
    dir.normalize();
    Eigen::Vector3f step = dir * step_size;
    for (int i = 0; i < step_num; ++i) {
      Eigen::Vector3f pos = lidar_map_interface_->ld_->lidar_pose_ + step * i;
      Eigen::Vector3i region_idx;
      getIndex(pos, region_idx);
      auto region = getRegionNode(region_idx);
      if (region != nullptr)
        region_set.insert(region);
    }
  }

  for (auto &region : region_set) {
    toponodes_update_region_arr_.push_back(region);
  }

  shorten_by_distance_insert_update_arr(toponodes_update_region_arr_);
  for (auto &region : toponodes_update_region_arr_) {
    update_idx_vec_.push_back(region->region_idx_);
  }
}

void TopoGraph::updateSkeleton() {
  parallel_bubble_astar_->reset();
  if (!node_admission_logged_) {
    ROS_INFO("[TopoGraph] node admission: safe_distance=%.3f, "
             "insert_margin=%.3f, insert_threshold=%.3f, "
             "clearance_tie_tolerance=%.3f",
             parallel_bubble_astar_->safe_distance_, node_insert_margin_,
             parallel_bubble_astar_->safe_distance_ + node_insert_margin_,
             node_clearance_tie_tolerance_);
    node_admission_logged_ = true;
  }
  vector<TopoNode::Ptr> nodes2insert, nodes_remained, nodes2remove, new_nodes, old_nodes;
  vector<TopologyNodeUpdateEvent> node_update_events;
  mutex new_nodes_mtx;
  ros::Time t0 = ros::Time::now();
  for (auto &region : toponodes_update_region_arr_) {
    for (auto &node : region->topo_nodes_) {
      if (!node->is_viewpoint_)
        old_nodes.push_back(node);
    }
  }
  omp_set_num_threads(6);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (auto &region_ptr : toponodes_update_region_arr_) {
    Eigen::Vector3f lb, hb;
    index2boundary(region_ptr->region_idx_, lb, hb);
    vector<BubbleNode::Ptr> tmp_bubbles;
    vector<bool> check_pt_flag(check_pts_.size(), false);
    for (int i = 0; i < check_pts_.size(); ++i) {
      Eigen::Vector3f pt = check_pts_[i].getArray3fMap();
      pt += lb;
      if (!lidar_map_interface_->IsInBox(pt))
        check_pt_flag[i] = true;
    }
    generateBubble(lb, hb, tmp_bubbles, check_pt_flag);
    BubbleUnionSet::Ptr union_set_ =
        std::make_shared<BubbleUnionSet>(
            parallel_bubble_astar_->safe_distance_ + node_insert_margin_,
            node_clearance_tie_tolerance_);
    vector<TopoNode::Ptr> new_nodes_region;
    Eigen::Vector3f region_center = (lb + hb) * 0.5;
    union_set_->unionSetCluster(tmp_bubbles, new_nodes_region, region_center);
    new_nodes_mtx.lock();
    for (auto &node : new_nodes_region) {
      if (!lidar_map_interface_->IsInBox(node->center_))
        continue;
      new_nodes.emplace_back(node);
    }
    new_nodes_mtx.unlock();
  }

  struct MatchCandidate {
    float distance;
    size_t new_idx;
    size_t old_idx;
  };
  vector<MatchCandidate> match_candidates;
  vector<int> new_match(new_nodes.size(), -1);
  vector<int> old_match(old_nodes.size(), -1);
  vector<float> nearest_new_distance(
      old_nodes.size(), std::numeric_limits<float>::infinity());
  vector<double> old_clearance(old_nodes.size(), 0.0);
  vector<char> old_clearance_ready(old_nodes.size(), 0);
  auto getOldClearance = [&](const size_t old_idx) {
    if (!old_clearance_ready[old_idx]) {
      old_clearance[old_idx] =
          lidar_map_interface_->getDisToOcc(old_nodes[old_idx]->center_);
      old_clearance_ready[old_idx] = 1;
    }
    return old_clearance[old_idx];
  };
  if (planner_debug_enabled_) {
    for (size_t old_idx = 0; old_idx < old_nodes.size(); ++old_idx) {
      for (const auto &new_node : new_nodes) {
        nearest_new_distance[old_idx] =
            std::min(nearest_new_distance[old_idx],
                     (new_node->center_ - old_nodes[old_idx]->center_).norm());
      }
    }
  }
  unordered_map<Eigen::Vector3i, vector<size_t>, Vector3iHash> old_buckets;
  const float match_resolution =
      std::max(0.001, node_match_tolerance_);
  auto bucketIndex = [&](const Eigen::Vector3f &center) {
    Eigen::Vector3i index =
        ((center - min_bd) / match_resolution)
            .array()
            .floor()
            .cast<int>()
            .matrix();
    return index;
  };
  for (size_t i = 0; i < old_nodes.size(); ++i)
    old_buckets[bucketIndex(old_nodes[i]->center_)].push_back(i);

  for (size_t new_idx = 0; new_idx < new_nodes.size(); ++new_idx) {
    const Eigen::Vector3i center_bucket =
        bucketIndex(new_nodes[new_idx]->center_);
    for (int dx = -1; dx <= 1; ++dx)
      for (int dy = -1; dy <= 1; ++dy)
        for (int dz = -1; dz <= 1; ++dz) {
          const Eigen::Vector3i bucket =
              center_bucket + Eigen::Vector3i(dx, dy, dz);
          auto bucket_it = old_buckets.find(bucket);
          if (bucket_it == old_buckets.end())
            continue;
          for (const size_t old_idx : bucket_it->second) {
            const float distance =
                (new_nodes[new_idx]->center_ - old_nodes[old_idx]->center_)
                    .norm();
            if (distance <= match_resolution)
              match_candidates.push_back(
                  MatchCandidate{distance, new_idx, old_idx});
          }
        }
  }
  std::sort(match_candidates.begin(), match_candidates.end(),
            [](const MatchCandidate &a, const MatchCandidate &b) {
              return a.distance < b.distance;
            });
  for (const auto &candidate : match_candidates) {
    if (new_match[candidate.new_idx] >= 0 ||
        old_match[candidate.old_idx] >= 0)
      continue;
    const double clearance = getOldClearance(candidate.old_idx);
    if (!std::isfinite(clearance) ||
        clearance <= parallel_bubble_astar_->safe_distance_)
      continue;
    new_match[candidate.new_idx] = candidate.old_idx;
    old_match[candidate.old_idx] = candidate.new_idx;
    if (planner_debug_enabled_) {
      auto &old_node = old_nodes[candidate.old_idx];
      node_update_events.push_back(TopologyNodeUpdateEvent{
          NODE_MATCHED, old_node, clearance,
          static_cast<int>(old_node->missed_update_count_),
          candidate.distance, old_node->neighbors_.size()});
    }
    old_nodes[candidate.old_idx]->missed_update_count_ = 0;
    nodes_remained.push_back(old_nodes[candidate.old_idx]);
  }

  for (size_t old_idx = 0; old_idx < old_nodes.size(); ++old_idx) {
    if (old_match[old_idx] >= 0)
      continue;
    auto &old_node = old_nodes[old_idx];
    const double clearance = getOldClearance(old_idx);
    const bool still_safe =
        std::isfinite(clearance) &&
        clearance > parallel_bubble_astar_->safe_distance_;
    const int missed_before =
        static_cast<int>(old_node->missed_update_count_);
    const double nearest_distance =
        std::isfinite(nearest_new_distance[old_idx])
            ? nearest_new_distance[old_idx]
            : -1.0;
    if (still_safe &&
        missed_before <
            std::max(0, node_miss_hysteresis_)) {
      if (planner_debug_enabled_) {
        node_update_events.push_back(TopologyNodeUpdateEvent{
            NODE_RETAINED_BY_HYSTERESIS, old_node, clearance, missed_before,
            nearest_distance, old_node->neighbors_.size()});
      }
      old_node->missed_update_count_++;
      nodes_remained.push_back(old_node);
    } else {
      if (planner_debug_enabled_) {
        const int action =
            still_safe ? NODE_REMOVED_MISS_EXPIRED : NODE_REMOVED_UNSAFE;
        node_update_events.push_back(TopologyNodeUpdateEvent{
            action, old_node, clearance, missed_before, nearest_distance,
            old_node->neighbors_.size()});
        ROS_WARN(
            "[TopoNodeLifecycle] action=%s id=%lu center=(%.3f,%.3f,%.3f) "
            "clearance=%.3f safe_distance=%.3f missed=%d hysteresis=%d "
            "nearest_new=%.3f match_tolerance=%.3f neighbors=%zu",
            still_safe ? "REMOVED_MISS_EXPIRED" : "REMOVED_UNSAFE",
            static_cast<unsigned long>(old_node->stable_id_),
            old_node->center_.x(), old_node->center_.y(),
            old_node->center_.z(), clearance,
            parallel_bubble_astar_->safe_distance_, missed_before,
            node_miss_hysteresis_, nearest_distance, node_match_tolerance_,
            old_node->neighbors_.size());
      }
      nodes2remove.push_back(old_node);
    }
  }
  const double node_insert_clearance =
      parallel_bubble_astar_->safe_distance_ + node_insert_margin_;
  for (size_t new_idx = 0; new_idx < new_nodes.size(); ++new_idx) {
    if (new_match[new_idx] >= 0)
      continue;

    auto &new_node = new_nodes[new_idx];
    const double clearance =
        lidar_map_interface_->getDisToOcc(new_node->center_);
    if (std::isfinite(clearance) && clearance >= node_insert_clearance) {
      nodes2insert.push_back(new_node);
      continue;
    }

    if (planner_debug_enabled_) {
      node_update_events.push_back(TopologyNodeUpdateEvent{
          NODE_REJECTED_ADMISSION, new_node, clearance, 0, -1.0, 0});
    }
  }

  ros::Time t1 = ros::Time::now();
  removeNodes(nodes2remove);
  ros::Time t2 = ros::Time::now();
  const bool edges_changed = updateRemainedConnections(nodes_remained);
  ros::Time t3 = ros::Time::now();
  insertNodes(nodes2insert);
  if (planner_debug_enabled_) {
    for (const auto &node : nodes2insert) {
      node_update_events.push_back(TopologyNodeUpdateEvent{
          NODE_INSERTED, node,
          lidar_map_interface_->getDisToOcc(node->center_), 0, -1.0,
          node->neighbors_.size()});
    }
  }
  ros::Time t4 = ros::Time::now();
  if (!nodes2remove.empty() || !nodes2insert.empty() || edges_changed)
    topology_revision_++;
  if (planner_debug_enabled_) {
    std_msgs::Float64MultiArray message;
    // Header: version, topology revision, event count, match tolerance,
    // safe distance, miss hysteresis, node insertion margin. Each event has:
    // action, stable id, x, y, z, clearance, missed count before update,
    // nearest regenerated-node distance, neighbor count.
    message.data.reserve(7 + node_update_events.size() * 9);
    message.data.push_back(2.0);
    message.data.push_back(static_cast<double>(topology_revision_));
    message.data.push_back(static_cast<double>(node_update_events.size()));
    message.data.push_back(node_match_tolerance_);
    message.data.push_back(parallel_bubble_astar_->safe_distance_);
    message.data.push_back(static_cast<double>(node_miss_hysteresis_));
    message.data.push_back(node_insert_margin_);
    for (const auto &event : node_update_events) {
      message.data.push_back(static_cast<double>(event.action));
      message.data.push_back(static_cast<double>(event.node->stable_id_));
      message.data.push_back(event.node->center_.x());
      message.data.push_back(event.node->center_.y());
      message.data.push_back(event.node->center_.z());
      message.data.push_back(event.clearance);
      message.data.push_back(static_cast<double>(event.missed_before));
      message.data.push_back(event.nearest_new_distance);
      message.data.push_back(static_cast<double>(event.neighbor_count));
    }
    topology_node_updates_pub_.publish(message);
  }
  publishStabilityDebug();
  vector<TopoNode::Ptr> unreachable_nodes;


}

void TopoGraph::publishStabilityDebug() {
  if (!planner_debug_enabled_)
    return;

  visualization_msgs::Marker nodes_marker;
  nodes_marker.header.frame_id = "world";
  nodes_marker.header.stamp = ros::Time::now();
  nodes_marker.ns = "topology_stability_nodes";
  nodes_marker.id = 0;
  nodes_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  nodes_marker.action = visualization_msgs::Marker::ADD;
  nodes_marker.pose.orientation.w = 1.0;
  nodes_marker.scale.x = 0.22;
  nodes_marker.scale.y = 0.22;
  nodes_marker.scale.z = 0.22;

  visualization_msgs::Marker failed_edges_marker;
  failed_edges_marker.header = nodes_marker.header;
  failed_edges_marker.ns = "topology_failed_edges";
  failed_edges_marker.id = 0;
  failed_edges_marker.type = visualization_msgs::Marker::LINE_LIST;
  failed_edges_marker.action = visualization_msgs::Marker::ADD;
  failed_edges_marker.pose.orientation.w = 1.0;
  failed_edges_marker.scale.x = 0.05;

  unordered_set<TopoNode::Ptr> visited_nodes;
  for (const auto &region_entry : reg_map_idx2ptr_) {
    for (const auto &node : region_entry.second->topo_nodes_) {
      if (!node || node->is_viewpoint_ || !visited_nodes.insert(node).second)
        continue;

      geometry_msgs::Point point;
      point.x = node->center_.x();
      point.y = node->center_.y();
      point.z = node->center_.z();
      nodes_marker.points.push_back(point);
      std_msgs::ColorRGBA node_color;
      node_color.a = 0.95;
      if (node->missed_update_count_ == 0) {
        node_color.r = 0.1;
        node_color.g = 0.9;
        node_color.b = 0.2;
      } else {
        node_color.r = 1.0;
        node_color.g = 0.75;
        node_color.b = 0.0;
      }
      nodes_marker.colors.push_back(node_color);

      for (const auto &failure_entry : node->edge_failures_) {
        const auto &other = failure_entry.first;
        if (!other ||
            !std::less<const TopoNode *>()(node.get(), other.get()))
          continue;
        geometry_msgs::Point other_point;
        other_point.x = other->center_.x();
        other_point.y = other->center_.y();
        other_point.z = other->center_.z();
        failed_edges_marker.points.push_back(point);
        failed_edges_marker.points.push_back(other_point);

        std_msgs::ColorRGBA edge_color;
        edge_color.a = 0.9;
        if (failure_entry.second.last_result ==
            ParallelBubbleAstar::TIME_OUT) {
          edge_color.r = 0.8;
          edge_color.g = 0.1;
          edge_color.b = 1.0;
        } else if (failure_entry.second.last_result == EDGE_COLLISION) {
          edge_color.r = 1.0;
          edge_color.g = 0.45;
          edge_color.b = 0.0;
        } else {
          edge_color.r = 1.0;
          edge_color.g = 0.1;
          edge_color.b = 0.1;
        }
        failed_edges_marker.colors.push_back(edge_color);
        failed_edges_marker.colors.push_back(edge_color);
      }
    }
  }

  topology_stability_nodes_pub_.publish(nodes_marker);
  topology_failed_edges_pub_.publish(failed_edges_marker);
}

void TopoGraph::updateOdomNode(Eigen::Vector3f &odom_pos, float &yaw) {
  struct PairPtrHash {
    std::size_t operator()(const std::pair<TopoNode::Ptr, TopoNode::Ptr> &p) const {
      return std::hash<TopoNode::Ptr>()(p.first) ^ std::hash<TopoNode::Ptr>()(p.second);
    }
  };

  Eigen::Vector3i idx;
  getIndex(lidar_map_interface_->ld_->lidar_pose_, idx);
  vector<TopoNode::Ptr> pre_nbrs;
  for (int i = -1; i <= 1; i++)
    for (int j = -1; j <= 1; j++)
      for (int k = -1; k <= 1; k++) {
        Eigen::Vector3i tmp_idx = idx;
        tmp_idx(0) = idx(0) + i;
        tmp_idx(1) = idx(1) + j;
        tmp_idx(2) = idx(2) + k;
        if (tmp_idx.x() == 0 && tmp_idx.y() == 0 && tmp_idx.z() != 0)
          continue;
        auto region = getRegionNode(tmp_idx);
        if (region) {
          for (auto &topo : region->topo_nodes_) {
            if (topo == odom_node_)
              continue;
            // if(topo->is_viewpoint_)
            //   continue;
            pre_nbrs.emplace_back(topo);
          }
        }
      }
  std::unordered_map<std::pair<TopoNode::Ptr, TopoNode::Ptr>, vector<Eigen::Vector3f>, PairPtrHash> edge2insert;
  mutex edge2insert_mtx;
  omp_set_num_threads(4);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (auto &nbr : pre_nbrs) {
    vector<Eigen::Vector3f> path;
    int res = parallel_bubble_astar_->search(odom_pos, nbr->center_, path, update_connection_timeout, true);
    if (res == ParallelBubbleAstar::REACH_END && parallel_bubble_astar_->collisionCheck_shortenPath(path)) {
      edge2insert_mtx.lock();
      edge2insert.insert({std::make_pair(odom_node_, nbr), path});
      edge2insert_mtx.unlock();
    }
  }
  if (edge2insert.empty())
    return;
  // 更新odom节点
  odom_node_->center_ = odom_pos;
  odom_node_->yaw_ = yaw;
  // if (edge2insert.size() > 0) {
  for (auto &nei : odom_node_->neighbors_) {
    nei->neighbors_.erase(odom_node_);
    nei->weight_.erase(odom_node_);
    nei->paths_.erase(odom_node_);
    nei->unreachable_nbrs_.erase(odom_node_);
    nei->edge_failures_.erase(odom_node_);
  }
  odom_node_->neighbors_.clear();
  odom_node_->weight_.clear();
  odom_node_->paths_.clear();
  odom_node_->unreachable_nbrs_.clear();
  odom_node_->edge_failures_.clear();
  for (auto &edge : edge2insert) {
    odom_node_->neighbors_.insert(edge.first.second);
    odom_node_->paths_.insert({edge.first.second, edge.second});
    double cost;
    // parallel_bubble_astar_->calculatePathCost(edge.second, cost);
    // odom_node_->weight_[edge.first.second] = cost;
    odom_node_->weight_[edge.first.second] = 0;
    // auto nbr = edge.first.second;
    // nbr->neighbors_.insert(odom_node_);
    // nbr->weight_[odom_node_] = cost;
    // vector<Eigen::Vector3f> path = edge.second;
    // std::reverse(path.begin(), path.end());
    // nbr->paths_[odom_node_] = path;
  }
  // }
}

void TopoGraph::removeNode(TopoNode::Ptr &node) {
  if (node == nullptr)
    return;
  Eigen::Vector3i region_idx;
  getIndex(node->center_, region_idx);
  auto region_node = getRegionNode(region_idx);
  if (region_node == nullptr) {
    debug_exit("TopoGraph::removeNodes :region_node == nullptr ");
  }
  region_node->topo_nodes_.erase(node);

  // nbrs
  for (auto &nbr : node->neighbors_) {
    nbr->neighbors_.erase(node);
    nbr->paths_.erase(node);
    nbr->weight_.erase(node);
    nbr->unreachable_nbrs_.erase(node);
    nbr->edge_failures_.erase(node);
  }
  for (auto &entry : node->edge_failures_)
    entry.first->edge_failures_.erase(node);
  node->unreachable_nbrs_.clear();
  node->edge_failures_.clear();
  node->neighbors_.clear();
  node->weight_.clear();
  node->paths_.clear();
}

void TopoGraph::insertNode(TopoNode::Ptr &new_node, vector<TopoNode::Ptr> &nbr_nodes, vector<vector<Eigen::Vector3f>> &paths) {
  Eigen::Vector3i region_idx;
  getIndex(new_node->center_, region_idx);
  auto region_node = getRegionNode(region_idx);
  if (region_node == nullptr) {
    debug_exit("TopoGraph::insertNodes :region_node == nullptr ");
  }
  region_node->topo_nodes_.insert(new_node);
  for (int i = 0; i < nbr_nodes.size(); i++) {
    new_node->neighbors_.insert(nbr_nodes[i]);
    nbr_nodes[i]->neighbors_.insert(new_node);
    auto path = paths[i];
    new_node->paths_.insert({nbr_nodes[i], path});
    std::reverse(path.begin(), path.end());
    nbr_nodes[i]->paths_.insert({new_node, path});
    double cost;
    parallel_bubble_astar_->calculatePathCost(path, cost);
    new_node->weight_[nbr_nodes[i]] = cost;
    nbr_nodes[i]->weight_[new_node] = cost;
  }
}



int TopoGraph::searchPathWithBoundary(const Eigen::Vector3f &start, const Eigen::Vector3f &end, double &time_out, vector<Eigen::Vector3f> &path) {
  Eigen::Vector3f bd_min, bd_max;
  for (int i = 0; i < 3; i++) {
    bd_min(i) = min(start(i), end(i));
    bd_max(i) = max(start(i), end(i));
  }
  bd_min -= Eigen::Vector3f(init_region_size_x_ / 2.0, init_region_size_y_ / 2.0, init_region_size_z_ / 2.0);
  bd_max += Eigen::Vector3f(init_region_size_x_ / 2.0, init_region_size_y_ / 2.0, init_region_size_z_ / 2.0);
  int res = parallel_bubble_astar_->search(start, end, path, time_out, false, false, bd_min, bd_max);
  return res;
}

double TopoGraph::getPathLength(const vector<TopoNode::Ptr> &topo_path) {
  vector<Eigen::Vector3f> path;
  for (int i = 0; i < topo_path.size() - 1; i++) {
    auto back = topo_path[i];
    auto front = topo_path[i + 1];
    for (auto &pt : back->paths_[front]) {
      path.emplace_back(pt);
    }
  }
  double length = 0.0;
  for (int i = 0; i < path.size() - 1; ++i)
    length += (path[i + 1] - path[i]).norm();
  return length;
}

bool TopoGraph::hasOverlapWithBox(const Eigen::Vector3f &low_bd, const Eigen::Vector3f &high_bd) {
  const static vector<Eigen::Vector3f> tmp_vec{{0, 0, 0}, {0, 0, 1}, {0, 1, 0}, {1, 0, 0}, {0, 1, 1}, {1, 0, 1}, {1, 1, 0}, {1, 1, 1}};
  for (auto &tmp : tmp_vec) {
    Eigen::Vector3f pt;
    for (int i = 0; i < 3; i++) {
      pt(i) = tmp(i) * low_bd(i) + (1 - tmp(i)) * high_bd(i);
    }
    if (lidar_map_interface_->IsInBox(pt))
      return true;
  }
  return false;
}
