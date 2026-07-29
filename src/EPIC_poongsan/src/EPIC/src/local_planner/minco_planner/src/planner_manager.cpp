/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2023-12-28 14:48:50
 * @LastEditTime: 2023-12-30 15:02:15
 * @Description:
 * @
 * @Copyright (c) 2023 by ning-zelin, All Rights Reserved.
 */
#include <algorithm>
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
#include <limits>
#include <math.h>
#include <random>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <boost/foreach.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <plan_manage/planner_manager.h>
#include <frontier_manager/frontier_manager.h>
#include <std_msgs/Int32.h>
#include <thread>
#include <visualization_msgs/Marker.h>

namespace {

struct GuidePathTarget {
  bool found = false;
  size_t segment_index = 0;
  double progress = 0.0;
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
};

bool isPointInsidePolytope(const Eigen::MatrixX4d &hPoly,
                           const Eigen::Vector3d &point,
                           const double tolerance) {
  if (hPoly.rows() == 0 || !hPoly.allFinite() || !point.allFinite())
    return false;

  const Eigen::Vector4d homogeneous(point.x(), point.y(), point.z(), 1.0);
  return (hPoly * homogeneous).maxCoeff() <= tolerance;
}

GuidePathTarget findFarthestGuidePathTarget(
    const std::vector<Eigen::Vector3d> &guidePath,
    const Eigen::MatrixX4d &hPoly, const double boundaryMargin) {
  GuidePathTarget best;
  if (guidePath.size() < 2 || hPoly.rows() == 0 || !hPoly.allFinite() ||
      !std::isfinite(boundaryMargin) || boundaryMargin < 0.0)
    return best;

  constexpr double kEps = 1.0e-9;
  double completedLength = 0.0;

  for (size_t segment = 0; segment + 1 < guidePath.size(); ++segment) {
    const Eigen::Vector3d start = guidePath[segment];
    const Eigen::Vector3d direction = guidePath[segment + 1] - start;
    const double segmentLength = direction.norm();
    if (!start.allFinite() || !direction.allFinite() ||
        !std::isfinite(segmentLength) || segmentLength <= kEps) {
      continue;
    }

    double enter = 0.0;
    double exit = 1.0;
    bool intersects = true;
    for (int row = 0; row < hPoly.rows(); ++row) {
      const Eigen::Vector3d normal =
          hPoly.row(row).head<3>().transpose();
      const double normalNorm = normal.norm();
      const double offset = hPoly(row, 3);
      if (!normal.allFinite() || !std::isfinite(normalNorm) ||
          !std::isfinite(offset)) {
        intersects = false;
        break;
      }

      if (normalNorm <= kEps) {
        if (offset > kEps)
          intersects = false;
        continue;
      }

      // Shrink the fallback target domain by a physical distance from every
      // corridor face. Plane normals are not assumed to be normalized.
      const double valueAtStart =
          normal.dot(start) + offset + boundaryMargin * normalNorm;
      const double slope = normal.dot(direction);
      if (std::abs(slope) <= kEps) {
        if (valueAtStart > kEps) {
          intersects = false;
          break;
        }
        continue;
      }

      const double crossing = -valueAtStart / slope;
      if (slope > 0.0)
        exit = std::min(exit, crossing);
      else
        enter = std::max(enter, crossing);

      if (enter > exit + kEps) {
        intersects = false;
        break;
      }
    }

    if (intersects && exit >= -kEps && enter <= 1.0 + kEps) {
      const double fraction = std::max(0.0, std::min(1.0, exit));
      const double progress = completedLength + fraction * segmentLength;
      if (!best.found || progress > best.progress + kEps) {
        best.found = true;
        best.segment_index = segment;
        best.progress = progress;
        best.point = start + fraction * direction;
      }
    }
    completedLength += segmentLength;
  }

  return best;
}

void truncateGuidePath(std::vector<Eigen::Vector3d> &guidePath,
                       const GuidePathTarget &target) {
  constexpr double kEps = 1.0e-9;
  if (!target.found || target.segment_index + 1 >= guidePath.size())
    return;

  std::vector<Eigen::Vector3d> truncated(
      guidePath.begin(), guidePath.begin() + target.segment_index + 1);
  if (truncated.empty() ||
      (target.point - truncated.back()).norm() > kEps) {
    truncated.push_back(target.point);
  }
  guidePath.swap(truncated);
}

} // namespace

namespace fast_planner {
// SECTION interfaces for setup and query

FastPlannerManager::FastPlannerManager() {}

FastPlannerManager::~FastPlannerManager() {
  lidar_map_interface_.reset();
  gcopter_viz_.reset();
  std::cout << "des manager" << std::endl;
}

void FastPlannerManager::printTimeCost(double time_threhold, double time_cost,
                                       string printInfo) {
  if (time_cost > time_threhold) {
    std::cout << "\033[31m " << printInfo << time_cost << " ms" << "\033[0m"
              << std::endl;
  } else {
    std::cout << "\033[32m " << printInfo << time_cost << " ms" << "\033[0m"
              << std::endl;
  }
}

void FastPlannerManager::publishDebugGuidePath(
    const vector<Eigen::Vector3d> &path) {
  if (!planner_debug_enabled_ || debug_guide_path_pub_.getNumSubscribers() == 0)
    return;

  nav_msgs::Path msg;
  msg.header.seq = static_cast<uint32_t>(current_debug_plan_seq_);
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  msg.poses.reserve(path.size());
  for (const auto &point : path) {
    geometry_msgs::PoseStamped pose;
    pose.header = msg.header;
    pose.pose.position.x = point.x();
    pose.pose.position.y = point.y();
    pose.pose.position.z = point.z();
    pose.pose.orientation.w = 1.0;
    msg.poses.push_back(pose);
  }
  debug_guide_path_pub_.publish(msg);
}

void FastPlannerManager::publishDebugObstaclePoints(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) {
  if (!planner_debug_enabled_ ||
      debug_obstacle_points_pub_.getNumSubscribers() == 0)
    return;

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  msg.header.seq = static_cast<uint32_t>(current_debug_plan_seq_);
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  debug_obstacle_points_pub_.publish(msg);
}

void FastPlannerManager::publishDebugHPolys(
    const std::vector<Eigen::MatrixX4d> &hPolys, ros::Publisher &publisher) {
  if (!planner_debug_enabled_ || publisher.getNumSubscribers() == 0)
    return;

  // Schema v1: [version, plan_seq, poly_count,
  //             rows, (nx, ny, nz, d) * rows, ...].
  std_msgs::Float64MultiArray msg;
  size_t value_count = 3;
  for (const auto &poly : hPolys)
    value_count += 1 + static_cast<size_t>(poly.rows()) * 4;
  msg.data.reserve(value_count);
  msg.data.push_back(1.0);
  msg.data.push_back(static_cast<double>(current_debug_plan_seq_));
  msg.data.push_back(static_cast<double>(hPolys.size()));
  for (const auto &poly : hPolys) {
    msg.data.push_back(static_cast<double>(poly.rows()));
    for (int row = 0; row < poly.rows(); ++row)
      for (int col = 0; col < 4; ++col)
        msg.data.push_back(poly(row, col));
  }
  publisher.publish(msg);
}

void FastPlannerManager::publishDebugTrajectoryClearance(
    bool safe, double collision_time, double min_distance,
    const Eigen::Vector3d &sample_position) {
  if (!planner_debug_enabled_ ||
      debug_traj_clearance_pub_.getNumSubscribers() == 0)
    return;

  // Schema v1: [version, plan_seq, traj_id, safe, collision_time,
  //             min_known_obstacle_distance, x, y, z, hard_radius].
  std_msgs::Float64MultiArray msg;
  msg.data = {
      1.0,
      static_cast<double>(current_debug_plan_seq_),
      static_cast<double>(local_data_.traj_id_),
      safe ? 1.0 : 0.0,
      collision_time,
      min_distance,
      sample_position.x(),
      sample_position.y(),
      sample_position.z(),
      gcopter_config_->dilateRadiusHard,
  };
  debug_traj_clearance_pub_.publish(msg);
}

void FastPlannerManager::initPlanModules(
    ros::NodeHandle &nh, ParallelBubbleAstar::Ptr &parallel_path_finder,
    TopoGraph::Ptr &graph) {

  local_data_.traj_id_ = 0;

  lidar_map_interface_ = graph->lidar_map_interface_;
  nh.getParam("max_traj_len", max_traj_len_);
  nh.getParam("lidar_perception/max_ray_length", max_ray_length);
  nh.getParam("lidar_perception/fov_up", fov_up);
  nh.getParam("lidar_perception/fov_down", fov_down);
  nh.getParam("lidar_perception/lidar_pitch", lidar_pitch);
  // ---- Local SFC corridor clipping to the observed FOV cone (Method B) ----
  nh.param("local_planning/clip_corridor_to_observed", clip_corridor_to_observed_, false);
  nh.param("local_planning/clip_cone_faces", clip_cone_faces_, false);
  nh.param("local_planning/p0_len_x", p0_len_x_, 0.6);
  nh.param("local_planning/p0_len_y", p0_len_y_, 0.6);
  nh.param("local_planning/p0_up", p0_up_, 0.2);
  nh.param("local_planning/p0_down", p0_down_, 0.2);
  nh.param("local_planning/viz_origin_corridor", viz_origin_corridor_, false);
  nh.param("planner_debug/enable", planner_debug_enabled_, false);
  double yaw_fov_deg = 360.0;
  nh.param("lidar_perception/yaw_fov", yaw_fov_deg, 360.0);
  yaw_fov_ = yaw_fov_deg * M_PI / 180.0;

  gcopter_viz_.reset(new Visualizer);
  gcopter_viz_->init(nh);
  gcopter_config_.reset(new GcopterConfig);
  gcopter_config_->init(nh);

  graph_visualizer_.reset(new GraphVisualizer);
  graph_visualizer_->init(nh);
  bubble_path_finder_.reset(new BubbleAstar);
  bubble_path_finder_->init(nh, lidar_map_interface_);
  topo_graph_ = graph;

  parallel_path_finder_ = parallel_path_finder;
  fast_searcher_.reset(new FastSearcher);
  fast_searcher_->init(topo_graph_, bubble_path_finder_);

  // odom 토픽은 config yaml 의 odometry_topic 에서만 읽는다. 구 하드코딩
  // 기본값(/aft_mapped_to_init) 폴백 제거 — 없으면 즉시 종료.
  string odom_topic;
  if (!nh.getParam("odometry_topic", odom_topic) || odom_topic.empty()) {
    ROS_FATAL("[PlannerManager] odometry_topic not set in config yaml. "
              "REFUSING TO START - no fallback.");
    ros::shutdown();
    exit(1);
  }

  pos_sub = nh.subscribe(odom_topic, 10,
                         &FastPlannerManager::posCallback, this);
  goal_sub = nh.subscribe("/move_base_simple/goal", 10,
                          &FastPlannerManager::goalCallback, this);
  yaw_state_pub = nh.advertise<std_msgs::Int32>("/yaw_state", 10);
  if (planner_debug_enabled_) {
    debug_obstacle_points_pub_ = nh.advertise<sensor_msgs::PointCloud2>(
        "/debug/local_planner_obstacle_points", 10);
    debug_guide_path_pub_ =
        nh.advertise<nav_msgs::Path>("/debug/local_guide_path", 10);
    debug_raw_hpolys_pub_ = nh.advertise<std_msgs::Float64MultiArray>(
        "/debug/raw_hpolys", 10);
    debug_clipped_hpolys_pub_ = nh.advertise<std_msgs::Float64MultiArray>(
        "/debug/clipped_hpolys", 10);
    debug_fov_faces_pub_ = nh.advertise<std_msgs::Float64MultiArray>(
        "/debug/active_fov_faces", 10);
    debug_traj_clearance_pub_ = nh.advertise<std_msgs::Float64MultiArray>(
        "/debug/trajectory_clearance", 100);
    ROS_WARN("[PlannerDebug] internal planner evidence publishers enabled");
  }
}

// test_gs
void FastPlannerManager::posCallback(const nav_msgs::OdometryConstPtr &msg) {

  const auto &q = msg->pose.pose.orientation;
  const double n2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (!std::isfinite(n2) ||
      n2 <= gcopter_config_->vectorNormEps *
                gcopter_config_->vectorNormEps) {
    ROS_WARN_THROTTLE(
        1.0, "[NumericalGuard] invalid odometry quaternion; keeping last yaw");
    return;
  }

  double roll, pitch;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(q, quat);
  quat.normalize();

  double yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  if (std::isfinite(yaw))
    local_data_.curr_yaw_ = yaw;
}

void FastPlannerManager::goalCallback(
    const geometry_msgs::PoseStampedConstPtr &msg) {
  const auto &q = msg->pose.orientation;
  const double n2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (!std::isfinite(n2) ||
      n2 <= gcopter_config_->vectorNormEps *
                gcopter_config_->vectorNormEps) {
    ROS_WARN_THROTTLE(10.0,
                      "[FastPlannerManager] goal with invalid quaternion "
                      "- ignoring yaw");
    return;
  }

  double roll, pitch;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg->pose.orientation, quat);
  quat.normalize();

  double yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  if (std::isfinite(yaw))
    local_data_.end_yaw_ = yaw;
}

bool FastPlannerManager::checkTrajVelocity() {
  ros::Time check_start = ros::Time::now();
  
  auto traj = local_data_.minco_traj_;
  double duration = local_data_.duration_;
  double curr_time = (ros::Time::now() - local_data_.start_time_).toSec();
  while (curr_time < duration) {
    Vector3d curr_vel = traj.getVel(curr_time);
    if (curr_vel.norm() > gcopter_config_->maxVelMag + 1.0) {
      return false;
    }
    curr_time += 0.3;
  }
  
  // Publish timing information
  double check_time = (ros::Time::now() - check_start).toSec() * 1000.0;
  gcopter_viz_->publishVelocityCheckCost(check_time);
  
  return true;
}

bool FastPlannerManager::checkTrajCollision(double &collision_time) {
  ros::Time check_start = ros::Time::now();
  
  PointType target;
  PointVector nearest_point;
  vector<float> PointDist;

  auto traj = local_data_.minco_traj_;
  double duration = local_data_.duration_;
  double curr_time = (ros::Time::now() - local_data_.start_time_).toSec();
  // collision_time is an absolute time on the current trajectory. Initialize it
  // on every path because a collision-free return previously left the caller
  // comparing an indeterminate stack value.
  collision_time = std::max(0.0, std::min(curr_time, duration));
  Vector3d last_sphere_cen_;
  if (curr_time > duration) {
    collision_time = duration;
    publishDebugTrajectoryClearance(
        true, collision_time, -1.0, local_data_.curr_pos_);
    return true;
  }

  last_sphere_cen_ = traj.getPos(curr_time);
  const double initial_distance =
      lidar_map_interface_->getDisToOcc(last_sphere_cen_);
  double min_distance =
      std::isfinite(initial_distance) && initial_distance >= 0.0
          ? initial_distance
          : std::numeric_limits<double>::infinity();
  Eigen::Vector3d min_distance_position = last_sphere_cen_;
  double last_radius_ = initial_distance - gcopter_config_->dilateRadiusHard;
  
  // Measure path collision check time
  ros::Time path_collision_start = ros::Time::now();
  
  while (curr_time < duration) {
    Vector3d curr_pos = traj.getPos(curr_time);
    if ((curr_pos - last_sphere_cen_).norm() < last_radius_) {
      curr_time += 0.05;
      continue;
    }
    target.x = curr_pos.x();
    target.y = curr_pos.y();
    target.z = curr_pos.z();
    lidar_map_interface_->KNN(target, 1, nearest_point, PointDist);
    if (PointDist.size() <= 0) {
      last_sphere_cen_ = curr_pos;
      last_radius_ = 4.0 - gcopter_config_->dilateRadiusHard;
      curr_time += 0.05;
      continue;
    }
    double dis2occ = sqrt(PointDist[0]);
    if (dis2occ < min_distance) {
      min_distance = dis2occ;
      min_distance_position = curr_pos;
    }
    if (dis2occ < gcopter_config_->dilateRadiusHard) {
      collision_time = curr_time;
      publishDebugTrajectoryClearance(
          false, collision_time, min_distance, min_distance_position);
      return false;
    }
    last_sphere_cen_ = curr_pos;
    last_radius_ = dis2occ - gcopter_config_->dilateRadiusHard;
    curr_time += 0.05;
  }
  
  ros::Time path_collision_end = ros::Time::now();
  double path_collision_time = (path_collision_end - path_collision_start).toSec() * 1000.0;
  gcopter_viz_->publishPathCollisionCheckCost(path_collision_time);
  
  // Publish timing information
  double check_time = (ros::Time::now() - check_start).toSec() * 1000.0;
  gcopter_viz_->publishCollisionCheckCost(check_time);
  publishDebugTrajectoryClearance(
      true, duration,
      std::isfinite(min_distance) ? min_distance : -1.0,
      min_distance_position);

  // No collision was found through the end of the trajectory.
  collision_time = duration;
  return true;
}

bool FastPlannerManager::planExploreTraj(const vector<Eigen::Vector3f> &path,
                                         bool is_static) {

  ros::Time start = ros::Time::now();
  if (planner_debug_enabled_)
    current_debug_plan_seq_ = ++planner_debug_seq_;
  if (path.size() < 2 ||
      !std::all_of(path.begin(), path.end(),
                   [](const Eigen::Vector3f &point) {
                     return point.allFinite();
                   })) {
    last_plan_fail_reason_ = "guide path has fewer than two finite points";
    ROS_WARN_THROTTLE(2.0, "[local-plan] %s",
                      last_plan_fail_reason_.c_str());
    return false;
  }

  vector<Eigen::Vector3d> path_shorten;
  bool use_shorten_path = false;
  int i = 0;
  int j = 0;
  for (j = path.size() - 1; j > 0; j--) {
    if ((path[j] - path[0]).norm() <= max_traj_len_ / 2.0)
      break;
  }
  double len = 0.0;
  for (i = 1; i < path.size();) {
    len += (path[i] - path[i - 1]).norm();
    if (len > max_traj_len_ || i == path.size() - 1) {
      break;
    }
    i++;
  }
  int end_idx = max(i, j);
  if (end_idx < path.size() - 1) {
    use_shorten_path = true;
  } else {
    use_shorten_path = false;
  }
  for (int i = 0; i <= end_idx; i++) {
    path_shorten.emplace_back(path[i].cast<double>());
  }

  // Keep both endpoints while removing interior guide points that would create
  // near-zero MINCO segments. The terminal point is preserved exactly.
  vector<Eigen::Vector3d> filtered_path;
  filtered_path.reserve(path_shorten.size());
  filtered_path.push_back(path_shorten.front());
  for (size_t idx = 1; idx + 1 < path_shorten.size(); ++idx) {
    if ((path_shorten[idx] - filtered_path.back()).norm() >=
        gcopter_config_->minPathSegmentLength) {
      filtered_path.push_back(path_shorten[idx]);
    }
  }
  const Eigen::Vector3d terminal = path_shorten.back();
  while (filtered_path.size() > 1 &&
         (terminal - filtered_path.back()).norm() <
             gcopter_config_->minPathSegmentLength) {
    filtered_path.pop_back();
  }
  filtered_path.push_back(terminal);
  if (filtered_path.size() != path_shorten.size()) {
    ROS_DEBUG("[NumericalGuard] compacted guide path from %zu to %zu points",
              path_shorten.size(), filtered_path.size());
  }
  path_shorten.swap(filtered_path);
  publishDebugGuidePath(path_shorten);

  if (use_shorten_path) {
    const Eigen::Vector3d fwd_dir =
        path_shorten.back() - path_shorten[path_shorten.size() - 2];
    if (fwd_dir.x() * fwd_dir.x() + fwd_dir.y() * fwd_dir.y() >
        fwd_dir.z() * fwd_dir.z()) {
      local_data_.end_yaw_ = atan2(fwd_dir.y(), fwd_dir.x());
    }
  }
  // 从小到大
  Eigen::Vector3f min_bd, max_bd;
  for (int i = 0; i < 3; i++) {
    min_bd[i] = path_shorten[0][i];
    max_bd[i] = path_shorten[0][i];
  }
  for (const Eigen::Vector3d &waypoint : path_shorten) {
    for (int i = 0; i < 3; i++) {
      if (waypoint[i] < min_bd[i]) {
        min_bd[i] = waypoint[i];
      }
      if (waypoint[i] > max_bd[i]) {
        max_bd[i] = waypoint[i];
      }
    }
  }
  for (int i = 0; i < 2; i++) {
    min_bd[i] =
        (min_bd[i] - gcopter_config_->corridorSearchMarginXY);
    max_bd[i] =
        (max_bd[i] + gcopter_config_->corridorSearchMarginXY);
  }
  min_bd[2] -= gcopter_config_->corridorSearchMarginZ;
  max_bd[2] += gcopter_config_->corridorSearchMarginZ;

  PointVector Searched_Points;
  lidar_map_interface_->boxSearch(min_bd, max_bd, Searched_Points);

  // 降采样
  std::vector<Eigen::Vector3d> surf_points;
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_origin(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(
      new pcl::PointCloud<pcl::PointXYZ>);
  cloud_origin->points = Searched_Points;
  sor.setInputCloud(cloud_origin);
  sor.setLeafSize(gcopter_config_->corridorObstacleVoxelSize,
                  gcopter_config_->corridorObstacleVoxelSize,
                  gcopter_config_->corridorObstacleVoxelSize);
  sor.filter(*cloud_tmp);
  publishDebugObstaclePoints(cloud_tmp);

  surf_points.reserve(cloud_tmp->points.size());
  for (const pcl::PointXYZ &point : cloud_tmp->points) {
    surf_points.emplace_back(point.x, point.y, point.z);
  }

  ros::Time point_process_end_stamp = ros::Time::now();

  std::vector<Eigen::MatrixX4d> hPolys; // 多面体飞行走廊

  sfc_gen::convexCover(
      gcopter_viz_, path_shorten, surf_points, min_bd.cast<double>(),
      max_bd.cast<double>(), gcopter_config_->corridorProgressLength,
      gcopter_config_->corridor_size, hPolys, 1e-6,
      gcopter_config_->dilateRadiusSoft,
      static_cast<size_t>(gcopter_config_->corridorMaxObstacleSamples),
      gcopter_config_->firiObstacleDistanceLimit,
      gcopter_config_->firiMaxPlaneCount,
      gcopter_config_->corridorGapViolationPlaneThreshold);

  // Preserve the raw FIRI corridor before the P0 seed box and observed-FOV
  // clipping. Method B does not alter the obstacle set, so no second
  // convexCover is needed for the comparison overlay.
  std::vector<Eigen::MatrixX4d> raw_hPolys;
  if (planner_debug_enabled_ ||
      (clip_corridor_to_observed_ && viz_origin_corridor_))
    raw_hPolys = hPolys;
  publishDebugHPolys(raw_hPolys, debug_raw_hpolys_pub_);

  // --- Prepend P0: a robot-sized, yaw-aligned free box at the current pose. The
  // space the robot physically occupies is free regardless of observation, so it
  // guarantees the start pose lies inside some polytope even when forward-FOV
  // frontier crowding degrades the first forward polytope (avoids the "current
  // position not in corridor" -> flyToSafeRegion escape in tight spaces). Fixed
  // to the robot's physical extent -- must NOT be grown into unobserved space.
  if (clip_corridor_to_observed_) {
    const Eigen::Vector3d p = local_data_.curr_pos_;
    const double psi = local_data_.curr_yaw_; // live odom yaw
    const double c = std::cos(psi), s = std::sin(psi);
    const Eigen::Vector3d bx(c, s, 0.0);  // body +x (forward)
    const Eigen::Vector3d by(-s, c, 0.0); // body +y (left)
    const double hx = 0.5 * p0_len_x_, hy = 0.5 * p0_len_y_;
    const double up = p0_up_, down = p0_down_;
    Eigen::MatrixX4d P0(6, 4); // rows (nx,ny,nz,d); constraint n.x + d <= 0
    P0.row(0) << bx.x(), bx.y(), 0.0, -(bx.dot(p) + hx);    // front
    P0.row(1) << -bx.x(), -bx.y(), 0.0, -(-bx.dot(p) + hx); // back
    P0.row(2) << by.x(), by.y(), 0.0, -(by.dot(p) + hy);    // left
    P0.row(3) << -by.x(), -by.y(), 0.0, -(-by.dot(p) + hy); // right
    P0.row(4) << 0.0, 0.0, 1.0, -(p.z() + up);              // up
    P0.row(5) << 0.0, 0.0, -1.0, (p.z() - down);            // down
    hPolys.insert(hPolys.begin(), P0);
  }

  // --- Method B: clip each forward polytope to the observed FOV cone (P0 kept).
  if (clip_corridor_to_observed_ && clip_cone_faces_ && frontier_manager_)
    clipCorridorToObservedCone(hPolys);

  Eigen::Matrix3d iniState;
  Eigen::Matrix3d finState;
  Eigen::Vector3d initialVelocity = local_data_.curr_vel_;
  if (!initialVelocity.allFinite()) {
    initialVelocity.setZero();
  } else if (initialVelocity.norm() > gcopter_config_->maxVelMag) {
    initialVelocity =
        initialVelocity.normalized() * gcopter_config_->maxVelMag;
  }
  // Position comes directly from the latest odometry. Velocity is only a soft
  // reference inside GCOPTER, and acceleration starts from a neutral reference.
  iniState << local_data_.curr_pos_, initialVelocity,
      Eigen::Vector3d::Zero();

  Eigen::Vector4d bh;
  bh << iniState.topLeftCorner<3, 1>(), 1.0;
  int start_idx = -1;
  for (int i = hPolys.size() - 1; i >= 0; i--) {
    Eigen::MatrixX4d hp = hPolys[i];
    if ((((hp * bh).array() > -1.0e-6).cast<int>().sum() <= 0)) {
      start_idx = i;
      break;
    }
  }
  if (start_idx == -1) {
    // 시작상태(예측점)가 corridor 밖: RTH 정체 사건의 주범. 이벤트 로거가 사유를
    // 읽어가므로 콘솔은 스로틀만 남긴다 (예전엔 이 ROS_ERROR가 초당 수십 번 스팸).
    last_plan_fail_reason_ =
        std::string("start not in corridor (") + (is_static ? "static" : "predicted") +
        " start; corridor pieces=" + std::to_string(hPolys.size()) + ")";
    ROS_WARN_THROTTLE(2.0, "[local-plan] %s", last_plan_fail_reason_.c_str());
    double safe_until = 0.0;
    const double elapsed_raw =
        (ros::Time::now() - local_data_.start_time_).toSec();
    const double elapsed =
        std::max(0.0, std::min(elapsed_raw, local_data_.duration_));
    const bool collision_free =
        local_data_.traj_id_ >= 1 && checkTrajCollision(safe_until);
    const double remaining_safe_time =
        std::max(0.0, safe_until - elapsed);

    // safe_until uses trajectory-relative absolute time. Subtract elapsed time
    // so the 2 s guard means "safe from now", rather than "trajectory duration
    // happens to be greater than 2 s".
    const bool can_keep_current_traj =
        collision_free && remaining_safe_time > 2.0;
    if (!can_keep_current_traj)
      return flyToSafeRegion(is_static);
    // return false;
  }
  if (start_idx > 0) { // -1(위에서 계속 진행하는 경우) erase 방지
    hPolys.erase(hPolys.begin(), hPolys.begin() + start_idx);
  }
  sfc_gen::shortCut(hPolys, gcopter_config_->corridorOverlapTolerance);

  // ros::Duration(1.0).sleep();

  ros::Time hpoly_gen_end = ros::Time::now();

  if (hPolys.size() < 2) {
    last_plan_fail_reason_ = "corridor pieces < 2 (free-space too tight around path)";
    ROS_WARN_THROTTLE(2.0, "[local-plan] %s", last_plan_fail_reason_.c_str());
    return false;
  }
  size_t lastConnected = 0;
  for (size_t index = 1; index < hPolys.size(); ++index) {
    if (!geo_utils::overlap(hPolys[index - 1], hPolys[index],
                            gcopter_config_->corridorOverlapTolerance)) {
      break;
    }
    lastConnected = index;
  }

  const bool chainFullyConnected = lastConnected + 1 == hPolys.size();
  const bool goalInsideLastCorridor =
      chainFullyConnected &&
      isPointInsidePolytope(hPolys.back(), path_shorten.back(), 1.0e-6);
  if (!chainFullyConnected || !goalInsideLastCorridor) {
    const GuidePathTarget fallback = findFarthestGuidePathTarget(
        path_shorten, hPolys[lastConnected],
        gcopter_config_->corridorFallbackTargetMargin);
    if (!fallback.found) {
      last_plan_fail_reason_ =
          "corridor fallback has no guide-path point inside connected prefix";
      ROS_WARN_THROTTLE(2.0, "[local-plan] %s",
                        last_plan_fail_reason_.c_str());
      return false;
    }
    if (fallback.progress <
        gcopter_config_->corridorFallbackMinProgress) {
      last_plan_fail_reason_ =
          "corridor fallback progress below configured minimum";
      ROS_WARN_THROTTLE(
          2.0, "[local-plan] %s (%.3f < %.3f m)",
          last_plan_fail_reason_.c_str(), fallback.progress,
          gcopter_config_->corridorFallbackMinProgress);
      return false;
    }

    const size_t originalCorridorCount = hPolys.size();
    hPolys.resize(lastConnected + 1);
    truncateGuidePath(path_shorten, fallback);
    if (path_shorten.size() < 2) {
      last_plan_fail_reason_ =
          "corridor fallback produced fewer than two guide-path points";
      ROS_WARN_THROTTLE(2.0, "[local-plan] %s",
                        last_plan_fail_reason_.c_str());
      return false;
    }

    use_shorten_path = true;
    const Eigen::Vector3d fwdDir =
        path_shorten.back() - path_shorten[path_shorten.size() - 2];
    if (fwdDir.x() * fwdDir.x() + fwdDir.y() * fwdDir.y() >
        fwdDir.z() * fwdDir.z()) {
      local_data_.end_yaw_ = std::atan2(fwdDir.y(), fwdDir.x());
    }
    finState << fallback.point, Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero();
    ROS_WARN_THROTTLE(
        1.0,
        "corridor fallback target selected at %.2f m guide progress "
        "(connected pieces %zu/%zu)",
        fallback.progress, lastConnected + 1, originalCorridorCount);
    gcopter_viz_->visualizePolytope(hPolys, true);
  } else {
    finState << path_shorten.back(), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero();
    gcopter_viz_->visualizePolytope(hPolys);
  }
  if (!raw_hPolys.empty())
    gcopter_viz_->visualizePolytopeOrigin(raw_hPolys);
  publishDebugHPolys(hPolys, debug_clipped_hpolys_pub_);

  gcopter::GCOPTER_PolytopeSFC gcopter;
  const Eigen::Vector2d magnitudeBounds(gcopter_config_->maxVelMag,
                                        gcopter_config_->maxAccMag);
  const Eigen::Vector3d penaltyWeights(
      gcopter_config_->pvaRunningWeights[0],
      gcopter_config_->pvaRunningWeights[1],
      gcopter_config_->pvaRunningWeights[2]);
  const Eigen::Vector2d initialWeights(
      gcopter_config_->pvaInitialWeights[0],
      gcopter_config_->pvaInitialWeights[1]);
  const Eigen::Vector3d terminalWeights(
      gcopter_config_->pvaTerminalWeights[0],
      gcopter_config_->pvaTerminalWeights[1],
      gcopter_config_->pvaTerminalWeights[2]);
  const int quadratureRes = gcopter_config_->integralIntervs;

  if (!gcopter.setup(
          gcopter_config_->weightT, gcopter_config_->dilateRadiusSoft, iniState,
          finState, hPolys, INFINITY, gcopter_config_->smoothingEps,
          quadratureRes, magnitudeBounds, penaltyWeights, initialWeights,
          terminalWeights,
          gcopter_config_->vectorNormEps,
          gcopter_config_->minSegmentTime,
          gcopter_config_->linearSolvePivotEps)) {
    last_plan_fail_reason_ = "gcopter setup failed (corridor/start-state infeasible)";
    ROS_WARN_THROTTLE(2.0, "[local-plan] %s", last_plan_fail_reason_.c_str());
    return false;
  }
  auto local_data_backup = local_data_;
  local_data_.minco_traj_.clear();

  // Measure trajectory generation time
  ros::Time traj_gen_start = ros::Time::now();
  const double position_cost =
      gcopter.optimize(local_data_.minco_traj_,
                       gcopter_config_->relCostTol);
  if (!std::isfinite(position_cost)) {
    last_plan_fail_reason_ = "minco optimize failed";
    ROS_WARN_THROTTLE(2.0, "[local-plan] %s", last_plan_fail_reason_.c_str());
    local_data_ = local_data_backup;
    return false;
  } else {
    local_data_.duration_ = local_data_.minco_traj_.getTotalDuration();
    if (!std::isfinite(local_data_.duration_) ||
        local_data_.duration_ < gcopter_config_->minSegmentTime) {
      last_plan_fail_reason_ = "minco produced invalid trajectory duration";
      ROS_WARN_THROTTLE(2.0, "[local-plan] %s",
                        last_plan_fail_reason_.c_str());
      local_data_ = local_data_backup;
      return false;
    }
  }
  ros::Time traj_gen_end = ros::Time::now();
  double traj_gen_time = (traj_gen_end - traj_gen_start).toSec() * 1000.0;
  gcopter_viz_->publishTrajectoryGenerationCost(traj_gen_time);
  
  // Measure LBFGS optimization time (this is part of trajectory generation)
  ros::Time lbfgs_start = ros::Time::now();
  // LBFGS optimization happens inside gcopter.optimize()
  ros::Time lbfgs_end = ros::Time::now();
  double lbfgs_time = (lbfgs_end - lbfgs_start).toSec() * 1000.0;
  gcopter_viz_->publishLbfgsOptimizationCost(lbfgs_time);
  
  double time = 10.0;
  if (!checkTrajCollision(time) && time < 1.0) {
    last_plan_fail_reason_ = "post-check: new traj collides within 1s";
    ROS_WARN_THROTTLE(2.0, "[local-plan] %s", last_plan_fail_reason_.c_str());
    local_data_ = local_data_backup;
    return false;
  }
  if (!checkTrajVelocity()) {
    last_plan_fail_reason_ = "post-check: velocity limit exceeded";
    ROS_WARN_THROTTLE(2.0, "[local-plan] %s", last_plan_fail_reason_.c_str());
    local_data_ = local_data_backup;
    return false;
  }
  if (local_data_.minco_traj_.getPieceNum() > 0) {
    if (!YawTrajOpt(local_data_.curr_yaw_, local_data_.end_yaw_, is_static,
                    use_shorten_path)) {
      last_plan_fail_reason_ = "yaw traj opt failed";
      ROS_WARN_THROTTLE(2.0, "[local-plan] %s", last_plan_fail_reason_.c_str());
      local_data_ = local_data_backup;
      return false;
    }
    gcopter_viz_->visualize(local_data_.minco_traj_,
                            gcopter_config_->maxVelMag);
  } else {
    local_data_ = local_data_backup;
    last_plan_fail_reason_ = "optimized traj empty";
    ROS_WARN_THROTTLE(2.0, "[local-plan] %s", last_plan_fail_reason_.c_str());
    return false;
  }
  ros::Time optimize_end_stamp = ros::Time::now();
  double trajOptimize_time =
      (optimize_end_stamp - hpoly_gen_end).toSec() * 1000;
  
  // Publish timing information
  double PolysGenerate_time = (hpoly_gen_end - point_process_end_stamp).toSec() * 1000;
  double pointCloudProcess_time = (point_process_end_stamp - start).toSec() * 1000;
  gcopter_viz_->visualizeTimeCost(PolysGenerate_time, trajOptimize_time, pointCloudProcess_time);
  
  local_data_.traj_id_ += 1;
  local_data_.start_time_ = hpoly_gen_end;
  local_data_.start_pos_ = path_shorten.front();
  local_data_.duration_ = local_data_.minco_traj_.getTotalDuration();
  last_plan_fail_reason_.clear();
  last_plan_was_escape_ = false;
  return true;
}

// [feature: cone-clip] Method B: intersect each forward corridor polytope with
// the observed FOV cone's side half-planes so the corridor cannot extend past
// what a limited-FOV sensor has actually seen. P0 (index 0, the robot-body seed
// box) is always kept. A face only clips if it is a real observation boundary
// (some FOV-edge frontier lies on it) and the observed surface does not continue
// past it (no DENSE neighbour just outside).
void FastPlannerManager::clipCorridorToObservedCone(
    std::vector<Eigen::MatrixX4d> &hPolys) {
  if (!frontier_manager_ || hPolys.size() < 2)
    return;

  const Eigen::Vector3d p = local_data_.curr_pos_;
  const double psi = local_data_.curr_yaw_;
  const double hy = 0.5 * yaw_fov_;        // half horizontal FOV [rad]
  const double R = max_ray_length;         // far-plane range [m]
  const Eigen::Vector3d fwd(std::cos(psi), std::sin(psi), 0.0);

  // FOV cone faces (4 sides): constraint n.x + d <= 0 is INSIDE the cone. All
  // pass through the apex p. (Far plane @ max_ray omitted -- it only gated on
  // distance frontiers and was near-inert; range is still capped below by R.)
  struct Face {
    Eigen::Vector3d n;
    double d;
  };
  const Eigen::Vector3d Z(0.0, 0.0, 1.0);
  std::vector<Face> faces;
  const double aL = psi + hy; // left FOV-edge azimuth
  Eigen::Vector3d nL(-std::sin(aL), std::cos(aL), 0.0);  // outward-left
  faces.push_back({nL, -nL.dot(p)});
  const double aR = psi - hy; // right FOV-edge azimuth
  Eigen::Vector3d nR(std::sin(aR), -std::cos(aR), 0.0);  // outward-right
  faces.push_back({nR, -nR.dot(p)});
  const double bU = (fov_up - lidar_pitch) * M_PI / 180.0;   // up-edge elevation
  Eigen::Vector3d nU = -std::sin(bU) * fwd + std::cos(bU) * Z;  // outward-up
  faces.push_back({nU, -nU.dot(p)});
  const double bD = (fov_down - lidar_pitch) * M_PI / 180.0; // down-edge elevation
  Eigen::Vector3d nD = std::sin(bD) * fwd - std::cos(bD) * Z;   // outward-down
  faces.push_back({nD, -nD.dot(p)});

  // Gate each face: ACTIVE only if some frontier lies on it AND no frontier on it
  // has a DENSE (already-observed) neighbor just OUTSIDE it. Any DENSE outside =>
  // the observed surface continues past this face => leave it (do not clip),
  // which preserves mobility through already-seen space.
  const double cs = frontier_manager_->getCellSize();
  const double eps = 1.5 * cs;
  std::vector<char> has_frontier(faces.size(), 0);
  std::vector<char> observed_outside(faces.size(), 0);

  // Iterate only this frame's FOV-edge frontier cells (already the cells lying on
  // the cone faces) instead of scanning every cluster cell.
  for (const auto &cc : frontier_manager_->fov_edge_cells_) {
    const Eigen::Vector3d c(cc.x, cc.y, cc.z);
    const Eigen::Vector3d rel = c - p;
    if (fwd.dot(rel) <= 0.0 || rel.norm() > R) // in front & within range
      continue;
    for (size_t fi = 0; fi < faces.size(); ++fi) {
      if (observed_outside[fi]) // face already decided (deactivate)
        continue;
      if (std::abs(faces[fi].n.dot(c) + faces[fi].d) > eps)
        continue; // c not on this face plane
      has_frontier[fi] = 1;
      for (int dx = -1; dx <= 1 && !observed_outside[fi]; ++dx)
        for (int dy = -1; dy <= 1 && !observed_outside[fi]; ++dy)
          for (int dz = -1; dz <= 1 && !observed_outside[fi]; ++dz) {
            if (!dx && !dy && !dz)
              continue;
            const Eigen::Vector3d nbr = c + cs * Eigen::Vector3d(dx, dy, dz);
            if (faces[fi].n.dot(nbr) + faces[fi].d <= 0.0)
              continue; // keep only OUTSIDE neighbors
            if (frontier_manager_->getCellState(nbr.cast<float>()) == DENSE)
              observed_outside[fi] = 1;
          }
    }
  }

  std::vector<int> active_idx;
  for (size_t fi = 0; fi < faces.size(); ++fi)
    if (has_frontier[fi] && !observed_outside[fi])
      active_idx.push_back((int)fi);

  if (planner_debug_enabled_ && debug_fov_faces_pub_.getNumSubscribers() > 0) {
    // Schema v1: [version, plan_seq, face_count,
    //             face_index, has_frontier, observed_outside, active,
    //             nx, ny, nz, d, ...].
    std_msgs::Float64MultiArray msg;
    msg.data.reserve(3 + faces.size() * 8);
    msg.data.push_back(1.0);
    msg.data.push_back(static_cast<double>(current_debug_plan_seq_));
    msg.data.push_back(static_cast<double>(faces.size()));
    for (size_t fi = 0; fi < faces.size(); ++fi) {
      const bool active = has_frontier[fi] && !observed_outside[fi];
      msg.data.push_back(static_cast<double>(fi));
      msg.data.push_back(has_frontier[fi] ? 1.0 : 0.0);
      msg.data.push_back(observed_outside[fi] ? 1.0 : 0.0);
      msg.data.push_back(active ? 1.0 : 0.0);
      msg.data.push_back(faces[fi].n.x());
      msg.data.push_back(faces[fi].n.y());
      msg.data.push_back(faces[fi].n.z());
      msg.data.push_back(faces[fi].d);
    }
    debug_fov_faces_pub_.publish(msg);
  }
  if (active_idx.empty())
    return;

  // Append active faces to every polytope except P0 (index 0). Guard each append
  // with a non-emptiness check so a cut that would empty a polytope is skipped
  // (keeps the corridor feasible; goal-outside-corridor fallback handles the rest).
  for (size_t hi = 1; hi < hPolys.size(); ++hi) {
    Eigen::MatrixX4d hp = hPolys[hi];
    for (int fi : active_idx) {
      Eigen::MatrixX4d cand(hp.rows() + 1, 4);
      cand.topRows(hp.rows()) = hp;
      cand.row(hp.rows()) << faces[fi].n.x(), faces[fi].n.y(), faces[fi].n.z(),
          faces[fi].d;
      Eigen::Vector3d interior;
      if (geo_utils::findInterior(cand, interior))
        hp = cand;
    }
    hPolys[hi] = hp;
  }
}

void FastPlannerManager::angleLimite(double &angle) {
  if (!std::isfinite(angle)) {
    ROS_WARN_THROTTLE(1.0,
                      "[NumericalGuard] non-finite yaw delta replaced by zero");
    angle = 0.0;
    return;
  }
  angle = std::remainder(angle, 2.0 * M_PI);
}

bool FastPlannerManager::YawTrajOpt(double &start_yaw, double &end_yaw,
                                    bool is_static, bool use_shorten_path) {
  ros::Time yaw_opt_start = ros::Time::now();
  
  Eigen::Matrix3d iniStateYaw, finStateYaw;
  Eigen::MatrixXd wpsYaw;
  Eigen::VectorXd opt_times_Yaw;
  double yaw_sp, yaw_sv(0.0), yaw_sa(0.0), yaw_ep(end_yaw);

  // Replanning starts from the latest odometry yaw. Subsequent waypoints add
  // wrapped shortest-angle deltas to this value, producing one continuous
  // unwrapped yaw sequence even when the measured angle crosses +/-pi.
  yaw_sp = start_yaw;
  angleLimite(yaw_sp);
  static double yaw_dur = 0.3;
  // double yaw_dur = local_data_.duration_ / 12.0;
  double fwd_time = gcopter_config_->yaw_time_fwd;
  vector<double> look_fwd_wp;
  look_fwd_wp.push_back(yaw_sp);
  for (double t = yaw_dur; t < local_data_.duration_ + yaw_dur; t += yaw_dur) {
    if (t > local_data_.duration_) {
      double delta = local_data_.end_yaw_ - look_fwd_wp.back();
      angleLimite(delta);
      look_fwd_wp.push_back(look_fwd_wp.back() + delta);
      break;
    }
    Eigen::Vector3d p_c = local_data_.minco_traj_.getPos(t);
    double t_fwd = t + fwd_time;
    Eigen::Vector3d p_f =
        local_data_.minco_traj_.getPos(min(t_fwd, local_data_.duration_));
    Eigen::Vector2d dir = p_f.head(2) - p_c.head(2);
    if (dir.norm() < fabs(p_f.z() - p_c.z()) || dir.norm() < 0.05) {
      // if (p_f.z() - p_c.z() > 0) {
      //   look_fwd_wp.push_back(look_fwd_wp.back());
      // } else
      // if (dir.norm() > 0.1) {
      //   double yaw = atan2(dir.y(), dir.x()) + min(M_PI, yaw_dur *
      //   gcopter_config_->yaw_max_vel); double delta_yaw = yaw -
      //   look_fwd_wp.back(); angleLimite(delta_yaw);
      //   look_fwd_wp.push_back(look_fwd_wp.back() + delta_yaw);
      // } else
      look_fwd_wp.push_back(
          look_fwd_wp.back() +
          min(M_PI, yaw_dur * gcopter_config_->yaw_max_vel * 0.6));
    } else {
      double yaw = atan2(dir.y(), dir.x());
      double delta_yaw = yaw - look_fwd_wp.back();
      angleLimite(delta_yaw);
      look_fwd_wp.push_back(look_fwd_wp.back() + delta_yaw);
    }
  }

  vector<double> wp;
  double delta_yaw_max = gcopter_config_->yaw_max_vel * yaw_dur;
  std::random_device rd;
  std::mt19937 gen(rd()); // 使用Mersenne Twister作为随机数引擎

  std::normal_distribution<double> dist(0, 2e-3);
  for (int i = 0; i < look_fwd_wp.size(); i++) {
    if (i == 0) {
      wp.push_back(look_fwd_wp[i]);
      continue;
    }
    double last_yaw = wp.back();
    double diff2end_yaw = local_data_.end_yaw_ - last_yaw;
    angleLimite(diff2end_yaw);
    double time2end_yaw = abs(diff2end_yaw) / gcopter_config_->yaw_max_vel;
    double time_last = local_data_.duration_ - i * yaw_dur;
    double next_yaw;
    if (time_last <= time2end_yaw)
      next_yaw = local_data_.end_yaw_ + dist(gen);
    else
      next_yaw = look_fwd_wp[i] + dist(gen);

    double delta_yaw = next_yaw - last_yaw;
    angleLimite(delta_yaw);
    if (delta_yaw < 0 && delta_yaw < -delta_yaw_max) {
      delta_yaw = -delta_yaw_max;
    } else if (delta_yaw > 0 && delta_yaw > delta_yaw_max) {
      delta_yaw = delta_yaw_max;
    }
    wp.push_back(wp.back() + delta_yaw);
  }
  double delta2end = local_data_.end_yaw_ - wp.back();
  angleLimite(delta2end);
  local_data_.end_yaw_ = wp.back() + delta2end;
  yaw_ep = local_data_.end_yaw_;
  iniStateYaw << Eigen::Vector3d(yaw_sp, 0.0, 0.0),
      Eigen::Vector3d(yaw_sv, 0.0, 0.0), Eigen::Vector3d(yaw_sa, 0.0, 0.0);
  finStateYaw << Eigen::Vector3d(yaw_ep, 0.0, 0.0), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero();

  gcopter::GCOPTER_PolytopeSFC gcopter_yaw;
  if (!gcopter_yaw.setup_yaw(gcopter_config_->yaw_rho_vis,
                             gcopter_config_->integralIntervs,
                             gcopter_config_->yawDiffEps,
                             gcopter_config_->minSegmentTime,
                             gcopter_config_->linearSolvePivotEps)) {
    cout << "setup_yaw failed!" << endl;
    return false;
  }
  int pieceNUM = wp.size() - 2;
  if (pieceNUM <= 1) {
    opt_times_Yaw.resize(2);
    opt_times_Yaw[0] = local_data_.duration_ / 2.0;
    opt_times_Yaw[1] = local_data_.duration_ / 2.0;
    wpsYaw.resize(3, 1);
    wpsYaw(0, 0) = (wp[0] + wp[1]) / 2.0;
    wpsYaw(1, 0) = 0.0;
    wpsYaw(2, 0) = 0.0;
    pieceNUM = 2;
  } else {
    // 干掉最后一个值
    opt_times_Yaw.resize(wp.size() - 2);
    for (int i = 0; i < wp.size() - 3; i++) {
      opt_times_Yaw[i] = yaw_dur;
    }
    opt_times_Yaw[wp.size() - 3] =
        local_data_.duration_ - (wp.size() - 3) * yaw_dur;
    wpsYaw.resize(3, wp.size() - 3);
    for (int i = 1; i < wp.size() - 2; i++) {
      wpsYaw(0, i - 1) = wp[i];
      wpsYaw(1, i - 1) = 0.0;
      wpsYaw(2, i - 1) = 0.0;
    }
  }
  for (int i = 0; i < opt_times_Yaw.size(); ++i) {
    if (!std::isfinite(opt_times_Yaw[i]) ||
        opt_times_Yaw[i] < gcopter_config_->minSegmentTime) {
      opt_times_Yaw[i] = gcopter_config_->minSegmentTime;
    }
  }
  // double dur_yaw = 0.0;
  // for (int i = 0; i < opt_times_Yaw.size(); i++) {
  //   dur_yaw += opt_times_Yaw[i];
  // }
  // cout << "dur_p= " << local_data_.duration_ << " dur_yaw= " << dur_yaw <<
  // endl; cout << "start yaw = " << iniStateYaw.col(0).transpose() << endl;
  // cout << "end yaw = " << finStateYaw.col(0).transpose() << endl;
  // cout << "wpsYaw = " << endl;
  // for (int i = 0; i < wpsYaw.cols(); i++) {
  //   cout << wpsYaw.col(i).transpose()(0) << " ";
  // }
  // cout << endl;

  local_data_.minco_yaw_traj_.clear();
  const double yaw_cost =
      gcopter_yaw.optimize_yaw(iniStateYaw, finStateYaw, pieceNUM, wpsYaw,
                               opt_times_Yaw, local_data_.minco_yaw_traj_);
  if (!std::isfinite(yaw_cost)) {
    std::cout << "optimize yaw failed!" << std::endl;
    return false;
  }
  const double yaw_duration =
      local_data_.minco_yaw_traj_.getTotalDuration();
  if (!std::isfinite(yaw_duration) ||
      yaw_duration < gcopter_config_->minSegmentTime) {
    std::cout << "yaw trajectory duration invalid!" << std::endl;
    return false;
  }
  
  ros::Time yaw_opt_end = ros::Time::now();
  double yaw_opt_time = (yaw_opt_end - yaw_opt_start).toSec() * 1000.0;
  gcopter_viz_->publishYawTrajectoryOptimizationCost(yaw_opt_time);
  
  return true;
}

bool FastPlannerManager::flyToSafeRegion(bool is_static) {
  Eigen::Vector3f min_bd, max_bd;
  for (int i = 0; i < 3; i++) {
    min_bd[i] = topo_graph_->odom_node_->center_[i] - 2.0;
    max_bd[i] = topo_graph_->odom_node_->center_[i] + 2.0;
  }
  PointVector Searched_Points;
  lidar_map_interface_->boxSearch(min_bd, max_bd, Searched_Points);
  std::vector<Eigen::Vector3d> surf_points;
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_origin(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(
      new pcl::PointCloud<pcl::PointXYZ>);
  cloud_origin->points = Searched_Points;
  sor.setInputCloud(cloud_origin);
  sor.setLeafSize(0.2, 0.2, 0.2);
  sor.filter(*cloud_tmp);

  surf_points.reserve(cloud_tmp->points.size());
  for (const pcl::PointXYZ &point : cloud_tmp->points) {
    surf_points.emplace_back(point.x, point.y, point.z);
  }
  Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
  bd(0, 0) = 1.0;
  bd(1, 0) = -1.0;
  bd(2, 1) = 1.0;
  bd(3, 1) = -1.0;
  bd(4, 2) = 1.0;
  bd(5, 2) = -1.0;
  bd(0, 3) =
      -(std::min(topo_graph_->odom_node_->center_(0) + 2.0f,
                 lidar_map_interface_->lp_->global_map_max_boundary_[0]));
  bd(1, 3) = std::max(topo_graph_->odom_node_->center_(0) - 2.0f,
                      lidar_map_interface_->lp_->global_box_min_boundary_[0]);
  bd(2, 3) =
      -(std::min(topo_graph_->odom_node_->center_(1) + 2.0f,
                 lidar_map_interface_->lp_->global_map_max_boundary_[1]));
  bd(3, 3) = std::max(topo_graph_->odom_node_->center_(1) - 2.0f,
                      lidar_map_interface_->lp_->global_box_min_boundary_[1]);
  bd(4, 3) =
      -(std::min(topo_graph_->odom_node_->center_(2) + 1.0f,
                 lidar_map_interface_->lp_->global_map_max_boundary_[2]));
  bd(5, 3) = std::max(topo_graph_->odom_node_->center_(2) - 1.0f,
                      lidar_map_interface_->lp_->global_box_min_boundary_[2]);
  Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pc(
      surf_points[0].data(), 3, surf_points.size());
  Eigen::MatrixX4d hp;
  firi::firi(bd, pc, topo_graph_->odom_node_->center_.cast<double>(),
             topo_graph_->odom_node_->center_.cast<double>(), hp, 2, 1.0e-6,
             gcopter_config_->firiObstacleDistanceLimit,
             gcopter_config_->firiMaxPlaneCount); // 计算出包含a和b的凸包
  std::vector<Eigen::MatrixX4d> hPolys;
  hPolys.push_back(hp);
  hPolys.push_back(hp);
  Eigen::Vector3d inner;
  geo_utils::findInterior(hp, inner);
  Eigen::Vector4d bh;
  double time_now = (ros::Time::now() - local_data_.start_time_).toSec();
  const Eigen::Vector3d inner_delta =
      inner - topo_graph_->odom_node_->center_.cast<double>();
  const double inner_delta_norm = inner_delta.norm();
  Eigen::Vector3d dir = Eigen::Vector3d::Zero();
  if (inner_delta.allFinite() &&
      inner_delta_norm > gcopter_config_->vectorNormEps) {
    dir = inner_delta / inner_delta_norm;
  }
  
  // Declare iniState variable
  Eigen::Matrix3d iniState;
  
  if (is_static) {
    // TSP、更新地图等会阻塞里程计回调函数，导致这里的数据不准，所以只有static才用
    iniState << local_data_.curr_pos_, dir * 0.2, Eigen::Vector3d::Zero();
    // iniState << topo_graph_->odom_node_->center_, local_data_.curr_vel_,
    // Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
  } else {
    time_now =
        time_now > local_data_.duration_ ? local_data_.duration_ : time_now;
    Eigen::Vector3d current_pose = local_data_.minco_traj_.getPos(time_now);
    iniState << current_pose, dir * 0.2, Eigen::Vector3d::Zero();
  }
  Eigen::Matrix3d finState;
  ros::Time hpoly_gen_end = ros::Time::now();
  // iniState << topo_graph_->odom_node_->center_, local_data_.curr_vel_,
  // Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
  finState << inner, dir, Eigen::Vector3d::Zero();
  bh << iniState.topLeftCorner<3, 1>(), 1.0;
  int start_idx = -1;
  for (int i = hPolys.size() - 1; i >= 0; i--) {
    Eigen::MatrixX4d hp = hPolys[i];
    if ((((hp * bh).array() > -1.0e-6).cast<int>().sum() <= 0)) {
      start_idx = i;
      break;
    }
  }
  if (start_idx == -1) {
    last_plan_fail_reason_ = "escape: start not in local free bubble";
    ROS_WARN_THROTTLE(2.0, "[local-plan] %s", last_plan_fail_reason_.c_str());
    return false;
  }
  if (start_idx > 0) {
    hPolys.erase(hPolys.begin(), hPolys.begin() + start_idx);
  }
  sfc_gen::shortCut(hPolys, gcopter_config_->corridorOverlapTolerance);

  // ros::Duration(1.0).sleep();

  if (hPolys.size() < 2) {
    last_plan_fail_reason_ = "escape: corridor pieces < 2";
    ROS_WARN_THROTTLE(2.0, "[local-plan] %s", last_plan_fail_reason_.c_str());
    return false;
  }
  gcopter_viz_->visualizePolytope(hPolys);
  gcopter::GCOPTER_PolytopeSFC gcopter;
  const Eigen::Vector2d magnitudeBounds(gcopter_config_->maxVelMag,
                                        gcopter_config_->maxAccMag);
  const Eigen::Vector3d penaltyWeights(
      gcopter_config_->pvaRunningWeights[0] * 2.0,
      gcopter_config_->pvaRunningWeights[1],
      gcopter_config_->pvaRunningWeights[2]);
  const Eigen::Vector2d initialWeights(
      gcopter_config_->pvaInitialWeights[0],
      gcopter_config_->pvaInitialWeights[1]);
  const Eigen::Vector3d terminalWeights(
      gcopter_config_->pvaTerminalWeights[0],
      gcopter_config_->pvaTerminalWeights[1],
      gcopter_config_->pvaTerminalWeights[2]);
  const int quadratureRes = gcopter_config_->integralIntervs;

  if (!gcopter.setup(
          gcopter_config_->WeightSafeT, gcopter_config_->dilateRadiusSoft,
          iniState, finState, hPolys, INFINITY, gcopter_config_->smoothingEps,
          quadratureRes, magnitudeBounds, penaltyWeights, initialWeights,
          terminalWeights,
          gcopter_config_->vectorNormEps,
          gcopter_config_->minSegmentTime,
          gcopter_config_->linearSolvePivotEps)) {
    last_plan_fail_reason_ = "escape: gcopter setup failed";
    ROS_WARN_THROTTLE(2.0, "[local-plan] %s", last_plan_fail_reason_.c_str());
    return false;
  }
  auto local_data_backup = local_data_;
  local_data_.minco_traj_.clear();
  const double escape_cost =
      gcopter.optimize(local_data_.minco_traj_,
                       gcopter_config_->relCostTol);
  if (!std::isfinite(escape_cost)) {
    last_plan_fail_reason_ = "escape: minco optimize failed";
    ROS_WARN_THROTTLE(2.0, "[local-plan] %s", last_plan_fail_reason_.c_str());
    local_data_ = local_data_backup;
    return false;
  }
  if (local_data_.minco_traj_.getPieceNum() > 0) {
    // ROS_INFO_STREAM(
    // "local_data_.minco_traj_.getPieceNum(): " <<
    // local_data_.minco_traj_.getPieceNum());
    gcopter_viz_->visualize(local_data_.minco_traj_,
                            gcopter_config_->maxVelMag);
  } else {
    local_data_ = local_data_backup;
    last_plan_fail_reason_ = "escape: optimized traj empty";
    ROS_WARN_THROTTLE(2.0, "[local-plan] %s", last_plan_fail_reason_.c_str());
    return false;
  }
  ros::Time optimize_end_stamp = ros::Time::now();
  local_data_.traj_id_ += 1;
  local_data_.start_time_ = hpoly_gen_end;
  local_data_.start_pos_ = topo_graph_->odom_node_->center_.cast<double>();
  local_data_.duration_ = local_data_.minco_traj_.getTotalDuration();
  if (!std::isfinite(local_data_.duration_) ||
      local_data_.duration_ < gcopter_config_->minSegmentTime) {
    local_data_ = local_data_backup;
    last_plan_fail_reason_ = "escape: invalid trajectory duration";
    return false;
  }
  // 성공: 단 이것은 요청 경로가 아니라 "안전지대 탈출" 궤적임을 표시
  last_plan_fail_reason_.clear();
  last_plan_was_escape_ = true;
  return true;
}

void FastPlannerManager::polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg,
                                         const ros::Time &start_time) {
  Eigen::VectorXd durs = local_data_.minco_traj_.getDurations();
  int piece_num = local_data_.minco_traj_.getPieceNum();
  poly_msg.drone_id = 0;
  poly_msg.traj_id = local_data_.traj_id_;
  poly_msg.start_time = start_time;
  poly_msg.order = 5;
  poly_msg.duration.resize(piece_num);
  poly_msg.coef_x.resize(6 * piece_num);
  poly_msg.coef_y.resize(6 * piece_num);
  poly_msg.coef_z.resize(6 * piece_num);
  for (int i = 0; i < piece_num; ++i) {
    poly_msg.duration[i] = durs(i);
    Eigen::Matrix<double, 3, 6> cMat =
        local_data_.minco_traj_.pieces[i].getCoeffMat();
    int i6 = i * 6;
    for (int j = 0; j < 6; j++) {
      poly_msg.coef_x[i6 + j] = cMat(0, j);
      poly_msg.coef_y[i6 + j] = cMat(1, j);
      poly_msg.coef_z[i6 + j] = cMat(2, j);
    }
  }
}

void FastPlannerManager::polyYawTraj2ROSMsg(traj_utils::PolyTraj &poly_msg,
                                            const ros::Time &start_time) {
  Eigen::VectorXd durs = local_data_.minco_yaw_traj_.getDurations();
  int piece_num = local_data_.minco_yaw_traj_.getPieceNum();
  poly_msg.drone_id = 0;
  poly_msg.traj_id = local_data_.traj_id_;
  poly_msg.start_time = start_time;
  poly_msg.order = 5;
  poly_msg.duration.resize(piece_num);
  poly_msg.coef_x.resize(6 * piece_num);
  poly_msg.coef_y.resize(6 * piece_num);
  poly_msg.coef_z.resize(6 * piece_num);
  for (int i = 0; i < piece_num; ++i) {
    poly_msg.duration[i] = durs(i);
    Eigen::Matrix<double, 3, 6> cMat =
        local_data_.minco_yaw_traj_.pieces[i].getCoeffMat();
    int i6 = i * 6;
    for (int j = 0; j < 6; j++) {
      poly_msg.coef_x[i6 + j] = cMat(0, j);
      poly_msg.coef_y[i6 + j] = cMat(1, j);
      poly_msg.coef_z[i6 + j] = cMat(2, j);
    }
  }
}

} // namespace fast_planner
