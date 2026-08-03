/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2024-03-01 15:11:38
 * @LastEditTime: 2024-03-05 17:26:18
 * @Description:
 * @
 * @Copyright (c) 2024 by ning-zelin, All Rights Reserved.
 */
#pragma once
#include <Eigen/Eigen>
#include <boost/functional/hash.hpp>
#include <iostream>
#include <map>
#include <pcl/common/distances.h>
#include <queue>
#include <ros/console.h>
#include <ros/ros.h>
#include <algorithm>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <vector>
#include <thread>
#include <lidar_map/lidar_map.h>
#include <unordered_map>
#include <unordered_set>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// #include <frontier_manager/frontier_manager.h>

using namespace fast_planner;

struct v3i_hash {
  std::size_t operator()(const Eigen::Vector3i &v) const {
    std::size_t seed = 0;
    seed ^= std::hash<int>()(v.x()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>()(v.y()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>()(v.z()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

// [feature: astar-profile] parallel_astar/*_timeout 튜닝을 위한 계측.
//
// 타임아웃은 "걸렸을 때만" 시간을 먹는다 — 0.5ms 에 끝나는 탐색은 상한이 4ms 든
// 20ms 든 여전히 0.5ms 다. 따라서 상한을 올리는 비용은
//     (타임아웃에 걸린 횟수) x (늘린 시간)
// 이 상한이고, 그 횟수를 모르면 "20ms 로 올리면 느려진다"는 판단을 할 수 없다.
// 그래서 탐색 1회마다 소요시간과 타임아웃 적중 여부를 모아 분포로 발행한다.
// 스레드 안전: search() 는 OpenMP 병렬 구간에서도 불리므로 기록은 락으로 보호한다.
struct AstarProfile {
  void record(double ms, bool hit_timeout);
  // 누적치를 사람이 읽는 한 줄로. reset=true 면 창을 비운다.
  std::string report(bool reset);
  size_t count() const { return samples_.size(); }

  std::mutex mtx_;
  std::vector<float> samples_;   // 소요시간 [ms]
  size_t timeouts_ = 0;
  double sum_ms_ = 0.0;
  double max_ms_ = 0.0;
};

class ParallelBubbleAstar {
public:
  enum { REACH_END = 1, NO_PATH = 2, START_FAIL = 3, END_FAIL = 4, TIME_OUT = 5 };

  // 모든 timeout 기반 탐색이 여기를 지난다 (연결 갱신 / 노드 삽입 / odom 노드).
  static AstarProfile profile_;

  struct Node {
    typedef ::shared_ptr<Node> Ptr;
    Ptr parent;
    Eigen::Vector3f position;
    double g_score;
    double f_score;
  };

  struct NodeCompre {
    bool operator()(Node::Ptr &node1, Node::Ptr &node2) { return node1->f_score > node2->f_score; }
  };

  typedef std::shared_ptr<ParallelBubbleAstar> Ptr;

  ros::Publisher open_set_pub_;
  // FrontierManager::Ptr frontier_manager_;
  double resolution_, inv_resolution_, lambda_heu_, safe_distance_, tie_breaker_;
  int allocate_num_;
  bool debug_;
  double max_vel_, max_acc_;
  LIOInterface::Ptr lidar_map_interface_;
  Eigen::Vector3f origin_;

  unordered_set<Eigen::Vector3i, v3i_hash> safe_node;
  unordered_set<Eigen::Vector3i, v3i_hash> dangerous_node;
  std::shared_timed_mutex safe_node_mtx;
  std::shared_timed_mutex dangerous_node_mtx;

  void posToIndex(const Eigen::Vector3f &pt, Eigen::Vector3i &idx);
  void IndexToPos(Eigen::Vector3f &pt, Eigen::Vector3i &idx);
  bool isNodeSafe(Node::Ptr node, const Eigen::Vector3f &bbox_min, const Eigen::Vector3f &bbox_max,
                  unordered_set<Eigen::Vector3i, v3i_hash> &safe_set, unordered_set<Eigen::Vector3i, v3i_hash> &danger_set);

  void init(ros::NodeHandle &nh, const LIOInterface::Ptr &lidar_map);
  void reset();
  bool collisionCheck_shortenPath(vector<Eigen::Vector3f> &path);
  // best_result = true: 启发式函数 = 1.0 * 欧氏距离
  // [feature: astar-profile] search() 는 계측 래퍼이고 실제 탐색은 searchImpl().
  // 반환점이 12개라 각 return 을 건드리는 대신 얇은 래퍼로 감쌌다.
  int search(const Eigen::Vector3f &start, const Eigen::Vector3f &goal, vector<Eigen::Vector3f> &path, double timeout, bool best_result = false,
             bool only_raycast = false,
             const Eigen::Vector3f &bbox_min = Eigen::Vector3f(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(),
                                                               std::numeric_limits<double>::lowest()),
             const Eigen::Vector3f &bbox_max = Eigen::Vector3f(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
                                                               std::numeric_limits<double>::max()));

  int searchImpl(const Eigen::Vector3f &start, const Eigen::Vector3f &goal, vector<Eigen::Vector3f> &path, double timeout, bool best_result,
                 bool only_raycast, const Eigen::Vector3f &bbox_min, const Eigen::Vector3f &bbox_max);
  void calculatePathCost(const vector<Eigen::Vector3f> &path, double &cost);
};