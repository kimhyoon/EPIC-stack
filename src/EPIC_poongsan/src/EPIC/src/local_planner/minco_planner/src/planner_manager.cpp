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

template <typename TrajectoryType>
bool findVirtualCeilingViolation(const TrajectoryType &trajectory,
                                 const double duration,
                                 const double ceilingZ,
                                 double &violationTime) {
  constexpr double kSampleStep = 0.02;
  constexpr double kBoundaryTolerance = 1.0e-6;
  violationTime = 0.0;
  if (!std::isfinite(duration) || duration < 0.0 ||
      !std::isfinite(ceilingZ)) {
    return true;
  }

  for (double time = 0.0; time < duration; time += kSampleStep) {
    const Eigen::Vector3d position = trajectory.getPos(time);
    if (!position.allFinite() ||
        position.z() > ceilingZ + kBoundaryTolerance) {
      violationTime = time;
      return true;
    }
  }

  const Eigen::Vector3d endPosition = trajectory.getPos(duration);
  violationTime = duration;
  return !endPosition.allFinite() ||
         endPosition.z() > ceilingZ + kBoundaryTolerance;
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
  // Independent of the two above: false disables CAUTION escape generation;
  // flyToSafeRegion never uses an unclipped fallback.
  nh.param("local_planning/clip_escape_to_observed", clip_escape_to_observed_,
           true);
  // Face-activation probe geometry. Read here rather than hardcoded so the
  // "variant A" widening (3.0 / 2) can be switched on from config once it has
  // been re-validated; the DEFAULTS deliberately reproduce the long-standing
  // behaviour (1.5 / 1) because variant A measured WORSE in the built code --
  // see the note in buildObservedBoundary().
  nh.param("local_planning/fov_face_tolerance_cells",
           fov_face_tolerance_cells_, 1.5);
  if (!std::isfinite(fov_face_tolerance_cells_) ||
      fov_face_tolerance_cells_ <= 0.0) {
    ROS_WARN("[PlannerManager] local_planning/fov_face_tolerance_cells=%.3f is "
             "not a positive finite number; falling back to 1.5",
             fov_face_tolerance_cells_);
    fov_face_tolerance_cells_ = 1.5;
  }
  nh.param("local_planning/fov_face_neighbor_radius",
           fov_face_neighbor_radius_, 1);
  if (fov_face_neighbor_radius_ < 1) {
    ROS_WARN("[PlannerManager] local_planning/fov_face_neighbor_radius=%d is "
             "below 1 (the probe would see nothing); falling back to 1",
             fov_face_neighbor_radius_);
    fov_face_neighbor_radius_ = 1;
  }
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
  const auto &position = msg->pose.pose.position;
  const auto &linear_velocity = msg->twist.twist.linear;
  const auto &angular_velocity = msg->twist.twist.angular;
  const double n2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z) || !std::isfinite(linear_velocity.x) ||
      !std::isfinite(linear_velocity.y) ||
      !std::isfinite(linear_velocity.z) ||
      !std::isfinite(angular_velocity.x) ||
      !std::isfinite(angular_velocity.y) ||
      !std::isfinite(angular_velocity.z) || !std::isfinite(n2) ||
      n2 <= gcopter_config_->vectorNormEps *
                gcopter_config_->vectorNormEps) {
    ROS_WARN_THROTTLE(
        1.0, "[NumericalGuard] invalid odometry state; keeping last state");
    return;
  }

  tf::Quaternion quat;
  tf::quaternionMsgToTF(q, quat);
  quat.normalize();

  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  const double cos_pitch = std::cos(pitch);
  if (!std::isfinite(roll) || !std::isfinite(pitch) ||
      !std::isfinite(yaw) || !std::isfinite(cos_pitch) ||
      std::abs(cos_pitch) <= gcopter_config_->vectorNormEps) {
    ROS_WARN_THROTTLE(
        1.0,
        "[NumericalGuard] odometry attitude is near the Euler singularity; "
        "keeping last state");
    return;
  }

  // nav_msgs/Odometry defines twist in child_frame_id. Rotate body-frame
  // linear velocity into the pose parent frame used by the position planner.
  const tf::Vector3 velocity_body(linear_velocity.x, linear_velocity.y,
                                  linear_velocity.z);
  const tf::Vector3 velocity_parent =
      tf::Matrix3x3(quat) * velocity_body;
  Eigen::Vector3d planner_velocity(velocity_parent.x(), velocity_parent.y(),
                                  velocity_parent.z());
  if (!planner_velocity.allFinite()) {
    ROS_WARN_THROTTLE(
        1.0,
        "[NumericalGuard] odometry velocity transform is invalid; keeping "
        "last state");
    return;
  }
  // angular.z is body-axis r, not generally the world-heading derivative.
  // Convert body q/r rates to Euler yaw rate before using it as a boundary
  // condition for the yaw polynomial.
  double yaw_rate =
      (std::sin(roll) * angular_velocity.y +
       std::cos(roll) * angular_velocity.z) /
      cos_pitch;
  if (!std::isfinite(yaw_rate)) {
    ROS_WARN_THROTTLE(
        1.0,
        "[NumericalGuard] converted odometry yaw rate is invalid; keeping "
        "last state");
    return;
  }
  {
    std::lock_guard<std::mutex> lock(latest_odom_mutex_);
    latest_odom_pva_.col(0) =
        Eigen::Vector3d(position.x, position.y, position.z);
    latest_odom_pva_.col(1) = planner_velocity;
    latest_odom_pva_.col(2).setZero();
    latest_odom_yaw_state_ << yaw, yaw_rate, 0.0;
    latest_odom_stamp_ =
        msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    latest_odom_state_available_ = true;
  }
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

bool FastPlannerManager::isRotateInPlaceHoldActive() const {
  if (!local_data_.rotate_in_place_ ||
      !std::isfinite(local_data_.duration_))
    return false;

  const double elapsed = (ros::Time::now() - local_data_.start_time_).toSec();
  return elapsed >= 0.0 && elapsed < local_data_.duration_;
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
  const double virtual_ceiling_z =
      lidar_map_interface_->lp_->global_box_max_boundary_.z() -
      gcopter_config_->corridorObstacleVoxelSize;
  Vector3d last_sphere_cen_;
  if (curr_time > duration) {
    collision_time = duration;
    publishDebugTrajectoryClearance(
        true, collision_time, -1.0, local_data_.curr_pos_);
    return true;
  }

  last_sphere_cen_ = traj.getPos(curr_time);
  if (!last_sphere_cen_.allFinite() ||
      last_sphere_cen_.z() > virtual_ceiling_z + 1.0e-6) {
    publishDebugTrajectoryClearance(
        false, collision_time, -1.0, last_sphere_cen_);
    return false;
  }
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
    if (!curr_pos.allFinite() ||
        curr_pos.z() > virtual_ceiling_z + 1.0e-6) {
      collision_time = curr_time;
      publishDebugTrajectoryClearance(
          false, collision_time, min_distance, curr_pos);
      return false;
    }
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

bool FastPlannerManager::checkCautionEscapeSafety(
    const Eigen::Vector3d &actual_position, double &violation_time,
    std::string &reason) {
  reason.clear();
  violation_time = 0.0;
  const LocalTrajData &escape = local_data_;
  constexpr double kSampleStep = 0.02;
  constexpr double kBoundaryTolerance = 1.0e-5;
  constexpr double kClearanceTolerance = 1.0e-3;

  auto fail = [&](const std::string &why, const double t) {
    reason = why;
    violation_time = t;
    return false;
  };
  if (!escape.caution_escape_ || escape.minco_traj_.getPieceNum() < 2 ||
      !std::isfinite(escape.duration_) || escape.duration_ <= 0.0 ||
      !std::isfinite(escape.escape_soft_entry_time_) ||
      escape.escape_soft_entry_time_ < 0.0 ||
      escape.escape_soft_entry_time_ > escape.duration_ ||
      !std::isfinite(escape.escape_virtual_ceiling_z_) ||
      !std::isfinite(escape.escape_hard_ceiling_z_) ||
      escape.escape_hard_ceiling_z_ < escape.escape_virtual_ceiling_z_ ||
      escape.escape_raw_polytope_.rows() <= 0 ||
      escape.escape_safe_polytope_.rows() <= 0 ||
      !escape.escape_raw_polytope_.allFinite() ||
      !escape.escape_safe_polytope_.allFinite()) {
    return fail("invalid escape safety certificate", 0.0);
  }

  const double elapsed = (ros::Time::now() - escape.start_time_).toSec();
  if (!std::isfinite(elapsed))
    return fail("non-finite escape execution time", 0.0);
  const bool execution_complete = elapsed >= escape.duration_;
  const double start_time =
      std::min(std::max(0.0, elapsed), escape.duration_);
  if (!actual_position.allFinite() ||
      actual_position.z() >
          escape.escape_hard_ceiling_z_ + kBoundaryTolerance)
    return fail("actual odometry violates hard escape ceiling", start_time);
  const Eigen::Vector4d actual_h(actual_position.x(), actual_position.y(),
                                 actual_position.z(), 1.0);
  if ((escape.escape_raw_polytope_ * actual_h).maxCoeff() >
      kBoundaryTolerance) {
    return fail("actual odometry left raw escape FIRI", start_time);
  }
  if (escape.escape_observed_halfspaces_.rows() > 0 &&
      (escape.escape_observed_halfspaces_ * actual_h).maxCoeff() >
          kBoundaryTolerance) {
    return fail("actual odometry left observed escape region", start_time);
  }

  const double actual_clearance =
      lidar_map_interface_->getDisToOcc(actual_position);
  if (!std::isfinite(actual_clearance) || actual_clearance < 0.0)
    return fail("invalid actual obstacle clearance", start_time);
  // Do not let trajectory expiry bypass the terminal safety conditions.  The
  // current-point exceptions exist only while actively moving out: once the
  // escape has ended, actual odometry must be inside both the virtual ceiling
  // and the configured soft obstacle margin.
  if (execution_complete) {
    if (actual_position.z() >
        escape.escape_virtual_ceiling_z_ + kBoundaryTolerance)
      return fail("completed escape remains above virtual ceiling", start_time);
    if (actual_clearance + kClearanceTolerance <
        gcopter_config_->dilateRadiusSoft)
      return fail("completed escape remains inside soft clearance", start_time);
    violation_time = escape.duration_;
    return true;
  }

  double clearance_floor = actual_clearance;
  bool reached_soft =
      actual_clearance + kClearanceTolerance >=
      gcopter_config_->dilateRadiusSoft;
  // Tracking can put the vehicle just outside the virtual safety ceiling even
  // though the published trajectory stayed below it.  That current state is
  // unavoidable; permit only a monotonically descending egress until the
  // virtual ceiling is re-entered, then make the ceiling strict again.  The
  // map/flight-volume ceiling remains a hard limit throughout.
  double ceiling_floor = actual_position.z();
  bool reached_virtual_ceiling =
      actual_position.z() <=
      escape.escape_virtual_ceiling_z_ + kBoundaryTolerance;
  for (double sample_time = start_time;; sample_time += kSampleStep) {
    const double t = std::min(sample_time, escape.duration_);
    const Eigen::Vector3d position = escape.minco_traj_.getPos(t);
    if (!position.allFinite() ||
        position.z() > escape.escape_hard_ceiling_z_ + kBoundaryTolerance)
      return fail("updated map check: invalid/hard-over-ceiling sample", t);
    if (!reached_virtual_ceiling) {
      if (position.z() > ceiling_floor + kBoundaryTolerance)
        return fail("updated map check: escape climbs above current ceiling violation",
                    t);
      ceiling_floor = std::min(ceiling_floor, position.z());
      reached_virtual_ceiling =
          position.z() <=
          escape.escape_virtual_ceiling_z_ + kBoundaryTolerance;
    } else if (position.z() >
               escape.escape_virtual_ceiling_z_ + kBoundaryTolerance) {
      return fail("updated map check: escape leaves virtual ceiling", t);
    }
    const Eigen::Vector4d position_h(position.x(), position.y(), position.z(),
                                     1.0);
    if ((escape.escape_raw_polytope_ * position_h).maxCoeff() >
        kBoundaryTolerance) {
      return fail("updated map check: escape leaves raw FIRI", t);
    }
    if (t + 1.0e-9 >= escape.escape_soft_entry_time_ &&
        (escape.escape_safe_polytope_ * position_h).maxCoeff() >
            kBoundaryTolerance) {
      return fail("updated map check: escape leaves soft FIRI", t);
    }
    if (escape.escape_observed_halfspaces_.rows() > 0 &&
        (escape.escape_observed_halfspaces_ * position_h).maxCoeff() >
            kBoundaryTolerance) {
      return fail("updated map check: escape leaves observed region", t);
    }

    const double clearance = lidar_map_interface_->getDisToOcc(position);
    if (!std::isfinite(clearance) || clearance < 0.0)
      return fail("updated map check: invalid obstacle clearance", t);
    if (!reached_soft) {
      if (clearance + kClearanceTolerance < clearance_floor)
        return fail("updated obstacle worsens egress clearance", t);
      clearance_floor = std::max(clearance_floor, clearance);
      reached_soft = clearance + kClearanceTolerance >=
                     gcopter_config_->dilateRadiusSoft;
    } else if (clearance + kClearanceTolerance <
               gcopter_config_->dilateRadiusSoft) {
      return fail("updated obstacle violates attained soft clearance", t);
    }
    if (t >= escape.duration_)
      break;
  }
  if (!reached_soft)
    return fail("updated map leaves no soft-clearance escape", escape.duration_);
  if (!reached_virtual_ceiling)
    return fail("updated trajectory never re-enters virtual ceiling",
                escape.duration_);

  violation_time = escape.duration_;
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

  // Take one atomic odometry snapshot for the complete planning attempt. P0
  // and the optimizer initial state must never be built from different odom
  // callbacks. Also reject a stale cloud/map pose instead of turning it into a
  // positional escape from the wrong location.
  Eigen::Matrix3d iniState;
  Eigen::Vector3d initialYawState;
  {
    std::lock_guard<std::mutex> lock(latest_odom_mutex_);
    if (!latest_odom_state_available_) {
      last_plan_fail_reason_ =
          "latest odometry state unavailable for replan initialization";
      ROS_WARN_THROTTLE(2.0, "[local-plan] %s",
                        last_plan_fail_reason_.c_str());
      return false;
    }
    iniState = latest_odom_pva_;
    initialYawState = latest_odom_yaw_state_;
  }
  const double map_odom_disagreement =
      (local_data_.curr_pos_ - iniState.col(0)).norm();
  if (!std::isfinite(map_odom_disagreement) || map_odom_disagreement > 0.5) {
    last_plan_fail_reason_ =
        "cloud/map odometry is stale relative to current odometry";
    ROS_ERROR_THROTTLE(
        1.0,
        "[local-plan] %s: map=(%.3f,%.3f,%.3f) odom=(%.3f,%.3f,%.3f) "
        "error=%.3f m",
        last_plan_fail_reason_.c_str(), local_data_.curr_pos_.x(),
        local_data_.curr_pos_.y(), local_data_.curr_pos_.z(),
        iniState(0, 0), iniState(1, 0), iniState(2, 0),
        map_odom_disagreement);
    return false;
  }

  // Reserve one obstacle-voxel layer at the top of the exploration box as an
  // exact vehicle-center ceiling. Keeping this independent of obstacle
  // dilation avoids applying the real-obstacle margin twice.
  const double virtual_ceiling_z =
      lidar_map_interface_->lp_->global_box_max_boundary_.z() -
      gcopter_config_->corridorObstacleVoxelSize;
  if (!std::isfinite(virtual_ceiling_z) ||
      virtual_ceiling_z <=
          lidar_map_interface_->lp_->global_box_min_boundary_.z() ||
      !iniState.col(0).allFinite() ||
      iniState(2, 0) > virtual_ceiling_z + 1.0e-6) {
    last_plan_fail_reason_ = "invalid or already violated virtual ceiling";
    ROS_WARN_THROTTLE(2.0, "[local-plan] %s (ceiling=%.3f, z=%.3f)",
                      last_plan_fail_reason_.c_str(), virtual_ceiling_z,
                      iniState(2, 0));
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
  if (path_shorten.front().z() > virtual_ceiling_z + 1.0e-6) {
    last_plan_fail_reason_ = "guide path starts above virtual ceiling";
    ROS_WARN_THROTTLE(2.0, "[local-plan] %s",
                      last_plan_fail_reason_.c_str());
    return false;
  }

  // The topology path may contain a node above the virtual ceiling. Preserve
  // its XY direction but stop the local Seed at the first line/ceiling
  // intersection. Never project the remaining high path downward because that
  // could create a new segment through an obstacle.
  for (size_t index = 1; index < path_shorten.size(); ++index) {
    if (path_shorten[index].z() <= virtual_ceiling_z + 1.0e-6)
      continue;

    const Eigen::Vector3d segment =
        path_shorten[index] - path_shorten[index - 1];
    if (!segment.allFinite() || segment.z() <= 1.0e-9) {
      last_plan_fail_reason_ =
          "guide path crosses virtual ceiling without a valid intersection";
      ROS_WARN_THROTTLE(2.0, "[local-plan] %s",
                        last_plan_fail_reason_.c_str());
      return false;
    }

    const double fraction =
        (virtual_ceiling_z - path_shorten[index - 1].z()) / segment.z();
    if (!std::isfinite(fraction) || fraction < 0.0 || fraction > 1.0) {
      last_plan_fail_reason_ =
          "guide path has an invalid virtual-ceiling intersection";
      ROS_WARN_THROTTLE(2.0, "[local-plan] %s",
                        last_plan_fail_reason_.c_str());
      return false;
    }

    const Eigen::Vector3d ceiling_endpoint =
        path_shorten[index - 1] + fraction * segment;
    path_shorten.resize(index);
    if ((ceiling_endpoint - path_shorten.back()).norm() >=
        gcopter_config_->minPathSegmentLength) {
      path_shorten.push_back(ceiling_endpoint);
    }
    use_shorten_path = true;
    break;
  }
  if (path_shorten.size() < 2) {
    last_plan_fail_reason_ =
        "virtual ceiling leaves fewer than two Seed points";
    ROS_WARN_THROTTLE(2.0, "[local-plan] %s",
                      last_plan_fail_reason_.c_str());
    return false;
  }
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

  // Evaluate the PR #1 observed boundary once per planning cycle. FIRI and the
  // post-clip validation consume this same immutable snapshot.
  ObservedBoundary observed_boundary;
  Eigen::MatrixX4d observed_halfspaces(0, 4);
  if (clip_corridor_to_observed_ && clip_cone_faces_ && frontier_manager_) {
    observed_boundary = buildObservedBoundary();
    observed_halfspaces = activeObservedHalfspaces(observed_boundary);
    publishObservedBoundaryDebug(observed_boundary);
  }

  Eigen::MatrixX4d corridor_halfspaces(observed_halfspaces.rows() + 1, 4);
  if (observed_halfspaces.rows() > 0)
    corridor_halfspaces.topRows(observed_halfspaces.rows()) =
        observed_halfspaces;
  corridor_halfspaces.row(observed_halfspaces.rows()) <<
      0.0, 0.0, 1.0, -virtual_ceiling_z;

  sfc_gen::convexCover(
      gcopter_viz_, path_shorten, surf_points, min_bd.cast<double>(),
      max_bd.cast<double>(), gcopter_config_->corridorProgressLength,
      gcopter_config_->corridor_size, hPolys, 1e-6,
      gcopter_config_->dilateRadiusSoft,
      static_cast<size_t>(gcopter_config_->corridorMaxObstacleSamples),
      gcopter_config_->firiObstacleDistanceLimit,
      gcopter_config_->firiMaxPlaneCount,
      gcopter_config_->corridorGapViolationPlaneThreshold,
      corridor_halfspaces);

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
    const Eigen::Vector3d p = iniState.col(0);
    const double psi = initialYawState.x(); // same live-odom snapshot as PVA
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
    P0.row(4) << 0.0, 0.0, 1.0,
        -std::min(p.z() + up, virtual_ceiling_z);             // up
    P0.row(5) << 0.0, 0.0, -1.0, (p.z() - down);            // down
    hPolys.insert(hPolys.begin(), P0);
  }

  // --- Method B: clip each forward polytope to the observed FOV cone (P0 kept).
  if (clip_corridor_to_observed_ && clip_cone_faces_ && frontier_manager_)
    clipCorridorToObservedBoundary(hPolys, observed_boundary);

  // Every replan starts from the latest measured odometry state. Do not
  // inherit a wall-clock sample from the previous command trajectory: when the
  // vehicle lags behind that command, inheritance moves each new trajectory
  // start farther away from the actual vehicle state.
  const ros::Time trajectory_start_time = ros::Time::now();
  Eigen::Matrix3d finState;

  const auto findStartCorridor =
      [&hPolys](const Eigen::Vector3d &position) -> int {
    if (!position.allFinite())
      return -1;
    Eigen::Vector4d homogeneous;
    homogeneous << position, 1.0;
    for (int index = static_cast<int>(hPolys.size()) - 1; index >= 0;
         --index) {
      if ((hPolys[index] * homogeneous).maxCoeff() <= 1.0e-6)
        return index;
    }
    return -1;
  };

  const int start_idx = findStartCorridor(iniState.col(0));

  if (start_idx == -1) {
    // 시작상태(odometry)가 corridor 밖: RTH 정체 사건의 주범. 이벤트 로거가 사유를
    // 읽어가므로 콘솔은 스로틀만 남긴다 (예전엔 이 ROS_ERROR가 초당 수십 번 스팸).
    last_plan_fail_reason_ = "odometry start not in corridor; corridor pieces=" +
                             std::to_string(hPolys.size());
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
    // [degrade-do-not-fail] The corridor does not reach the goal. Instead of
    // giving up, go as far as the corridor legally allows and let yaw rotate
    // toward the goal, which swings the FOV cone and can reopen the corridor on
    // the next cycle. Failing here is what produced the 57.7 s PLAN_TRAJ_RTH
    // deadlock in flight bag 0804/1970-01-01-09-10-25: the single-polytope
    // lookup below returned nothing, the plan returned false, and because yaw
    // is only planned AFTER a successful position trajectory the vehicle could
    // never turn to look where it needed to go.
    //
    // Walk BACKWARDS from the last connected piece toward the vehicle and take
    // the FIRST (i.e. highest-index, maximum-progress) piece that yields a
    // target. Index 0 is the piece that contains the odometry start, so the
    // walk terminates on something that contains the vehicle itself; if even
    // that is refused by the boundary margin, retry the whole walk with a zero
    // margin and finally fall back to holding position. corridorFallbackMinProgress
    // is now a PREFERENCE, not a hard failure.
    const double fallbackMargin =
        gcopter_config_->corridorFallbackTargetMargin;
    size_t fallbackIndex = lastConnected;
    double usedMargin = fallbackMargin;
    GuidePathTarget fallback;
    for (size_t attempt = 0; attempt < 2 && !fallback.found; ++attempt) {
      usedMargin = (attempt == 0) ? fallbackMargin : 0.0;
      for (size_t index = lastConnected + 1; index-- > 0;) {
        const GuidePathTarget candidate =
            findFarthestGuidePathTarget(path_shorten, hPolys[index],
                                        usedMargin);
        if (candidate.found) {
          fallback = candidate;
          fallbackIndex = index;
          break;
        }
      }
    }
    const bool degenerate_hold = !fallback.found;
    if (degenerate_hold) {
      fallback.found = true;
      fallback.segment_index = 0;
      fallback.progress = 0.0;
      fallback.point = path_shorten.front();
      fallbackIndex = 0;
    }
    const bool below_min_progress =
        fallback.progress < gcopter_config_->corridorFallbackMinProgress;

    // Direction we WANTED to travel, taken from the guide path BEFORE
    // truncation. truncateGuidePath keeps [0 .. segment_index] plus the target,
    // so for a heavily truncated path the post-truncation direction is already
    // the first guide segment; this pre-computed value is the fallback for the
    // degenerate cases where that difference collapses to (near) zero and
    // atan2 would return a meaningless 0.
    double desired_yaw = local_data_.end_yaw_;
    bool desired_yaw_valid = false;
    for (size_t index = 1; index < path_shorten.size(); ++index) {
      const Eigen::Vector3d dir = path_shorten[index] - path_shorten.front();
      if (dir.head<2>().squaredNorm() >
          std::max(dir.z() * dir.z(), 1.0e-6)) {
        desired_yaw = std::atan2(dir.y(), dir.x());
        desired_yaw_valid = true;
        break;
      }
    }

    // [rotate-in-place] The corridor admits no forward motion at all: the best
    // target is the start point itself. Do NOT hand that to gcopter -- a
    // zero-length problem is a non-problem and setup() rightly refuses it
    // ("gcopter setup failed", measured 6/6 on the hold cases). Synthesize both
    // trajectories directly instead: hold position, and slew yaw to the
    // direction we could not travel in, so the FOV cone swings and the next
    // cycle can find a real corridor. Mirror of the constant-yaw PolyTraj that
    // CAUTION hand-builds in fast_exploration_fsm.cpp:944-956.
    const double target_step = (fallback.point - iniState.col(0)).norm();
    constexpr double kUsefulStep = 1.0e-3; // [m]
    // A degenerate fallback is semantically HOLD-IN-PLACE even when the guide
    // path's first point and the replanning initial state differ. That mismatch
    // was measured at 0.6007 m in MARSIM, so checking target_step alone sent a
    // zero-progress problem to gcopter and failed forever instead of rotating.
    if (degenerate_hold || !std::isfinite(target_step) ||
        target_step < kUsefulStep) {
      const double yaw0 = initialYawState.x();
      double yaw_delta = (desired_yaw_valid ? desired_yaw : yaw0) - yaw0;
      angleLimite(yaw_delta); // shortest signed rotation, (-pi, pi]

      // Quintic smoothstep: zero yaw rate AND acceleration at both ends, so the
      // traj_server yaw limiter never has to absorb a step. Its peak rate is
      // 1.875*|d|/T, hence the duration floor below; clamped so a tiny delta
      // still yields a usable trajectory and a large one does not stall.
      constexpr double kMinHoldTime = 0.3; // [s]
      constexpr double kMaxHoldTime = 8.0; // [s]
      const double yaw_rate_limit =
          (std::isfinite(gcopter_config_->yaw_max_vel) &&
           gcopter_config_->yaw_max_vel > 1.0e-6)
              ? gcopter_config_->yaw_max_vel
              : 1.0;
      double hold_T = 1.875 * std::abs(yaw_delta) / yaw_rate_limit;
      if (!std::isfinite(hold_T))
        hold_T = kMinHoldTime;
      hold_T = std::max(kMinHoldTime, std::min(kMaxHoldTime, hold_T));

      // Coefficients are HIGHEST ORDER FIRST: col(0) is t^5 ... col(5) is the
      // constant (Piece::getPos walks i = D..0 with increasing powers,
      // trajectory.hpp:60-68), and polyTraj2ROSMsg copies cMat(row, j) straight
      // into coef_*[i*6 + j], so the published message keeps the same layout.
      Piece<5>::CoefficientMat pos_coeff = Piece<5>::CoefficientMat::Zero();
      pos_coeff.col(5) = iniState.col(0); // constant position, all rows
      Piece<5>::CoefficientMat yaw_coeff = Piece<5>::CoefficientMat::Zero();
      const double t2 = hold_T * hold_T;
      const double t3 = t2 * hold_T;
      const double t4 = t3 * hold_T;
      const double t5 = t4 * hold_T;
      yaw_coeff(0, 0) = 6.0 * yaw_delta / t5;   // t^5
      yaw_coeff(0, 1) = -15.0 * yaw_delta / t4; // t^4
      yaw_coeff(0, 2) = 10.0 * yaw_delta / t3;  // t^3
      yaw_coeff(0, 5) = yaw0;                   // constant; only x carries yaw
                                                // (traj_server get_yaw reads
                                                // yaw_traj_->getPos(t).x())

      const std::vector<double> durations(1, hold_T);
      local_data_.minco_traj_ = Trajectory<5>(
          durations, std::vector<Piece<5>::CoefficientMat>(1, pos_coeff));
      local_data_.minco_yaw_traj_ = Trajectory<5>(
          durations, std::vector<Piece<5>::CoefficientMat>(1, yaw_coeff));
      local_data_.end_yaw_ = yaw0 + yaw_delta;

      // Same bookkeeping as the normal success path below.
      local_data_.traj_id_ += 1;
      local_data_.start_time_ = trajectory_start_time;
      local_data_.start_pos_ = iniState.col(0);
      local_data_.duration_ = local_data_.minco_traj_.getTotalDuration();
      local_data_.rotate_in_place_ = true;
      local_data_.caution_escape_ = false;
      last_plan_fail_reason_.clear();
      last_plan_was_escape_ = false;

      hPolys.resize(fallbackIndex + 1);
      gcopter_viz_->visualizePolytope(hPolys, true);
      gcopter_viz_->visualize(local_data_.minco_traj_,
                              local_data_.minco_yaw_traj_,
                              trajectory_start_time,
                              gcopter_config_->maxVelMag);
      ROS_WARN_THROTTLE(
          1.0,
          "[local-plan] corridor admits no progress -- rotate in place: "
          "yaw %.1f -> %.1f deg over %.2f s (peak %.2f rad/s, limit %.2f)",
          yaw0 * 180.0 / M_PI, local_data_.end_yaw_ * 180.0 / M_PI, hold_T,
          1.875 * std::abs(yaw_delta) / hold_T, yaw_rate_limit);
      return true;
    }

    const size_t originalCorridorCount = hPolys.size();
    hPolys.resize(fallbackIndex + 1);
    truncateGuidePath(path_shorten, fallback);

    use_shorten_path = true;
    // path_shorten can legitimately hold a single point here (target == start),
    // so guard the [size-2] access instead of failing the plan.
    bool end_yaw_set = false;
    if (path_shorten.size() >= 2) {
      const Eigen::Vector3d fwdDir =
          path_shorten.back() - path_shorten[path_shorten.size() - 2];
      if (fwdDir.x() * fwdDir.x() + fwdDir.y() * fwdDir.y() >
          std::max(fwdDir.z() * fwdDir.z(), 1.0e-6)) {
        local_data_.end_yaw_ = std::atan2(fwdDir.y(), fwdDir.x());
        end_yaw_set = true;
      }
    }
    if (!end_yaw_set && desired_yaw_valid)
      local_data_.end_yaw_ = desired_yaw;

    finState << fallback.point, Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero();
    if (degenerate_hold || below_min_progress) {
      ROS_WARN_THROTTLE(
          1.0,
          "[local-plan] degraded corridor fallback: %.3f m progress "
          "(< %.3f m minimum%s), piece %zu/%zu, margin %.2f m, end_yaw %.1f deg "
          "-- holding/creeping and rotating toward the goal",
          fallback.progress, gcopter_config_->corridorFallbackMinProgress,
          degenerate_hold ? ", HOLD-IN-PLACE" : "", fallbackIndex + 1,
          originalCorridorCount, usedMargin,
          local_data_.end_yaw_ * 180.0 / M_PI);
    } else {
      ROS_WARN_THROTTLE(
          1.0,
          "corridor fallback target selected at %.2f m guide progress "
          "(connected pieces %zu/%zu)",
          fallback.progress, fallbackIndex + 1, originalCorridorCount);
    }
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
    last_plan_fail_reason_ = std::string("gcopter setup failed: ") +
                             gcopter.lastSetupError();
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
  double ceiling_violation_time = 0.0;
  if (findVirtualCeilingViolation(local_data_.minco_traj_,
                                  local_data_.duration_,
                                  virtual_ceiling_z,
                                  ceiling_violation_time)) {
    last_plan_fail_reason_ =
        "post-check: trajectory exceeds virtual ceiling";
    ROS_WARN_THROTTLE(2.0, "[local-plan] %s at t=%.3f (ceiling=%.3f)",
                      last_plan_fail_reason_.c_str(),
                      ceiling_violation_time, virtual_ceiling_z);
    local_data_ = local_data_backup;
    return false;
  }
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
    if (!YawTrajOpt(initialYawState, local_data_.end_yaw_,
                    use_shorten_path)) {
      last_plan_fail_reason_ = "yaw traj opt failed";
      ROS_WARN_THROTTLE(2.0, "[local-plan] %s", last_plan_fail_reason_.c_str());
      local_data_ = local_data_backup;
      return false;
    }
    const Eigen::Vector3d positionBoundaryError(
        (local_data_.minco_traj_.getPos(0.0) - iniState.col(0)).norm(),
        (local_data_.minco_traj_.getVel(0.0) - iniState.col(1)).norm(),
        (local_data_.minco_traj_.getAcc(0.0) - iniState.col(2)).norm());
    const Eigen::Vector3d yawBoundaryError(
        std::abs(local_data_.minco_yaw_traj_.getPos(0.0).x() -
                 initialYawState.x()),
        std::abs(local_data_.minco_yaw_traj_.getVel(0.0).x() -
                 initialYawState.y()),
        std::abs(local_data_.minco_yaw_traj_.getAcc(0.0).x() -
                 initialYawState.z()));
    ROS_INFO_THROTTLE(
        1.0,
        "[ReplanInitialState] source=odometry PVA_err=[%.3e %.3e %.3e] "
        "Yaw_err=[%.3e %.3e %.3e]",
        positionBoundaryError.x(), positionBoundaryError.y(),
        positionBoundaryError.z(), yawBoundaryError.x(),
        yawBoundaryError.y(), yawBoundaryError.z());
    // trajectory_start_time is what local_data_.start_time_ is set to below;
    // the FOV-frustum fade is keyed off it (planned time, not odometry).
    gcopter_viz_->visualize(local_data_.minco_traj_,
                            local_data_.minco_yaw_traj_, trajectory_start_time,
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
  local_data_.start_time_ = trajectory_start_time;
  local_data_.start_pos_ = iniState.col(0);
  local_data_.duration_ = local_data_.minco_traj_.getTotalDuration();
  local_data_.rotate_in_place_ = false;
  local_data_.caution_escape_ = false;
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
// Build the exact PR #1 boundary decision without mutating a Corridor. The
// resulting snapshot is shared by FIRI generation and the post-clip check.
FastPlannerManager::ObservedBoundary
FastPlannerManager::buildObservedBoundary() const {
  ObservedBoundary boundary;
  if (!frontier_manager_)
    return boundary;

  const Eigen::Vector3d p = local_data_.curr_pos_;
  const double psi = local_data_.curr_yaw_;
  const double hy = 0.5 * yaw_fov_;
  const double range = max_ray_length;
  const Eigen::Vector3d fwd(std::cos(psi), std::sin(psi), 0.0);
  boundary.apex = p;
  boundary.max_range = range;

  // Four candidate faces pass through the current pose. The far plane remains
  // omitted, matching PR #1 Method B.
  const Eigen::Vector3d z_axis(0.0, 0.0, 1.0);
  boundary.faces.reserve(4);
  const double left_angle = psi + hy;
  const Eigen::Vector3d left_normal(
      -std::sin(left_angle), std::cos(left_angle), 0.0);
  boundary.faces.push_back(
      {left_normal, -left_normal.dot(p), false, false, false});

  const double right_angle = psi - hy;
  const Eigen::Vector3d right_normal(
      std::sin(right_angle), -std::cos(right_angle), 0.0);
  boundary.faces.push_back(
      {right_normal, -right_normal.dot(p), false, false, false});

  const double up_angle = (fov_up - lidar_pitch) * M_PI / 180.0;
  const Eigen::Vector3d up_normal =
      -std::sin(up_angle) * fwd + std::cos(up_angle) * z_axis;
  boundary.faces.push_back(
      {up_normal, -up_normal.dot(p), false, false, false});

  const double down_angle = (fov_down - lidar_pitch) * M_PI / 180.0;
  const Eigen::Vector3d down_normal =
      std::sin(down_angle) * fwd - std::cos(down_angle) * z_axis;
  boundary.faces.push_back(
      {down_normal, -down_normal.dot(p), false, false, false});

  // Preserve PR #1 gating exactly: a face is active only when an FOV-edge
  // frontier lies on it and no DENSE neighbor continues outside it.
  //
  // The probe geometry (on-plane band, neighbour radius) is parameterised but
  // DEFAULTS TO THE ORIGINAL 1.5 cells / radius 1. A widening to 3.0 / 2
  // ("variant A") was proposed and then MEASURED TO BE WORSE, so it is not the
  // default; keeping it reachable from config only so it can be re-tested.
  //
  // Why it looked good and was not: an offline sweep over flight bag
  // 0804/1970-01-01-09-10-25 predicted the RTH-deadlock window (bag t
  // 683.9-706.0) would go from 74.3% to 0.0% UP-face activation. That sweep had
  // to approximate fov_edge_cells_ with the published /exploration_node/frt
  // cloud, which is a far denser set. Re-run live against this code, widening
  // moved the same window the WRONG way: UP active 31-39% -> 97.2%. Cause: with
  // the real (sparse) fov_edge_cells_, a wider band mostly turns has_frontier ON
  // for plans that previously had no on-plane cell at all, while the wider
  // neighbourhood still finds no DENSE outside -- so more plans end up
  // has_frontier && !observed_outside. Do not re-enable without instrumenting
  // fov_edge_cells_ itself.
  const double cell_size = frontier_manager_->getCellSize();
  const double face_tolerance = fov_face_tolerance_cells_ * cell_size;
  const int neighbor_radius = fov_face_neighbor_radius_;
  for (const auto &cell : frontier_manager_->fov_edge_cells_) {
    const Eigen::Vector3d center(cell.x, cell.y, cell.z);
    const Eigen::Vector3d relative = center - p;
    if (fwd.dot(relative) <= 0.0 || relative.norm() > range)
      continue;

    for (ObservedBoundaryFace &face : boundary.faces) {
      if (face.observed_outside)
        continue;
      if (std::abs(face.normal.dot(center) + face.offset) >
          face_tolerance)
        continue;

      face.has_frontier = true;
      // Neighbour half-width is local_planning/fov_face_neighbor_radius (2 by
      // default, was 1) -- see the variant-A note above the tolerance.
      for (int dx = -neighbor_radius;
           dx <= neighbor_radius && !face.observed_outside; ++dx)
        for (int dy = -neighbor_radius;
             dy <= neighbor_radius && !face.observed_outside; ++dy)
          for (int dz = -neighbor_radius;
               dz <= neighbor_radius && !face.observed_outside; ++dz) {
            if (!dx && !dy && !dz)
              continue;
            const Eigen::Vector3d neighbor =
                center + cell_size * Eigen::Vector3d(dx, dy, dz);
            if (face.normal.dot(neighbor) + face.offset <= 0.0)
              continue;
            // DENSE only, deliberately. SPARSE was measured and REJECTED: on
            // bag 0804/1970-01-01-09-10-25 every DENSE||SPARSE variant drops
            // the takeoff-hover anchor to 0% active (0/28 frames, and 0/24 on
            // 1970-01-01-09-06-08), i.e. it would let the corridor reach into
            // rear space the vehicle has NEVER observed, immediately after
            // takeoff. That is the failure this face exists to prevent, so do
            // not widen this test to SPARSE.
            if (frontier_manager_->getCellState(neighbor.cast<float>()) ==
                DENSE)
              face.observed_outside = true;
          }
    }
  }

  for (ObservedBoundaryFace &face : boundary.faces)
    face.active = face.has_frontier && !face.observed_outside;

  return boundary;
}

Eigen::MatrixX4d FastPlannerManager::activeObservedHalfspaces(
    const ObservedBoundary &boundary) const {
  const size_t active_count =
      std::count_if(boundary.faces.begin(), boundary.faces.end(),
                    [](const ObservedBoundaryFace &face) {
                      return face.active;
                    });
  Eigen::MatrixX4d halfspaces(active_count, 4);
  size_t row = 0;
  for (const ObservedBoundaryFace &face : boundary.faces) {
    if (!face.active)
      continue;
    halfspaces.row(row++) << face.normal.x(), face.normal.y(),
        face.normal.z(), face.offset;
  }
  return halfspaces;
}

void FastPlannerManager::publishObservedBoundaryDebug(
    const ObservedBoundary &boundary) {
  if (!planner_debug_enabled_ ||
      debug_fov_faces_pub_.getNumSubscribers() == 0)
    return;

  // Keep the existing debug schema unchanged.
  std_msgs::Float64MultiArray msg;
  msg.data.reserve(3 + boundary.faces.size() * 8);
  msg.data.push_back(1.0);
  msg.data.push_back(static_cast<double>(current_debug_plan_seq_));
  msg.data.push_back(static_cast<double>(boundary.faces.size()));
  for (size_t index = 0; index < boundary.faces.size(); ++index) {
    const ObservedBoundaryFace &face = boundary.faces[index];
    msg.data.push_back(static_cast<double>(index));
    msg.data.push_back(face.has_frontier ? 1.0 : 0.0);
    msg.data.push_back(face.observed_outside ? 1.0 : 0.0);
    msg.data.push_back(face.active ? 1.0 : 0.0);
    msg.data.push_back(face.normal.x());
    msg.data.push_back(face.normal.y());
    msg.data.push_back(face.normal.z());
    msg.data.push_back(face.offset);
  }
  debug_fov_faces_pub_.publish(msg);
}

// Keep PR #1 post-clipping as a validation and compatibility layer. P0 remains
// exempt and a face that would empty a polytope is skipped, exactly as before.
//
// The corridor is now allowed to be clipped FULLY, even when that removes the
// guide path and the goal from every forward polytope. A guard used to skip any
// face that cut the covered guide path, because losing coverage made
// planExploreTraj fail outright -- the 57.7 s PLAN_TRAJ_RTH deadlock in flight
// bag 0804/1970-01-01-09-10-25. That was the wrong trade: it bought liveness by
// silently dropping the observed-space constraint and letting the corridor
// extend past what the sensor has actually seen. planExploreTraj now degrades
// the local goal instead (walks back through the corridor to the farthest
// admissible point and rotates yaw toward the unreachable goal), so an
// unreachable goal no longer needs the constraint relaxed. Go only as far as we
// legally can; never further.
void FastPlannerManager::clipCorridorToObservedBoundary(
    std::vector<Eigen::MatrixX4d> &hPolys,
    const ObservedBoundary &boundary) {
  if (hPolys.size() < 2)
    return;

  for (size_t poly_index = 1; poly_index < hPolys.size(); ++poly_index) {
    Eigen::MatrixX4d poly = hPolys[poly_index];
    for (const ObservedBoundaryFace &face : boundary.faces) {
      if (!face.active)
        continue;
      Eigen::MatrixX4d candidate(poly.rows() + 1, 4);
      candidate.topRows(poly.rows()) = poly;
      candidate.row(poly.rows()) << face.normal.x(), face.normal.y(),
          face.normal.z(), face.offset;
      Eigen::Vector3d interior;
      if (geo_utils::findInterior(candidate, interior))
        poly.swap(candidate);
    }
    hPolys[poly_index] = poly;
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

bool FastPlannerManager::YawTrajOpt(
    const Eigen::Vector3d &initial_yaw_state, double &end_yaw,
    bool use_shorten_path) {
  ros::Time yaw_opt_start = ros::Time::now();
  
  Eigen::Matrix3d iniStateYaw, finStateYaw;
  Eigen::MatrixXd wpsYaw;
  Eigen::VectorXd opt_times_Yaw;
  double yaw_sp(initial_yaw_state.x()), yaw_sv(initial_yaw_state.y()),
      yaw_sa(initial_yaw_state.z()), yaw_ep(end_yaw);

  // Keep the inherited yaw unwrapped. Wrapping this absolute start angle would
  // reintroduce a +/-pi discontinuity even though all following deltas already
  // use the shortest angular distance.
  if (!initial_yaw_state.allFinite()) {
    ROS_WARN_THROTTLE(
        1.0, "[ReplanContinuity] invalid initial yaw state");
    return false;
  }
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
  bool used_initial_yaw_fallback = false;
  const double yaw_cost =
      gcopter_yaw.optimize_yaw(iniStateYaw, finStateYaw, pieceNUM, wpsYaw,
                               opt_times_Yaw, local_data_.minco_yaw_traj_,
                               &used_initial_yaw_fallback);
  if (!std::isfinite(yaw_cost)) {
    std::cout << "optimize yaw failed!" << std::endl;
    return false;
  }
  if (used_initial_yaw_fallback) {
    ROS_WARN_THROTTLE(
        2.0,
        "[local-plan] yaw LBFGS line search failed; using finite initial yaw "
        "trajectory (%d pieces, %.2f s)",
        pieceNUM, opt_times_Yaw.sum());
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
  // Use the same exact vehicle-center ceiling as normal local planning.  The
  // old escape bubble was capped only by map bounds (often several metres
  // above the exploration box), so its FIRI interior could command a climb out
  // of the legal flight volume.
  const double virtual_ceiling_z =
      lidar_map_interface_->lp_->global_box_max_boundary_.z() -
      gcopter_config_->corridorObstacleVoxelSize;
  const double hard_ceiling_z =
      lidar_map_interface_->lp_->global_box_max_boundary_.z();
  if (!std::isfinite(virtual_ceiling_z) ||
      !std::isfinite(hard_ceiling_z) ||
      hard_ceiling_z < virtual_ceiling_z ||
      virtual_ceiling_z <=
          lidar_map_interface_->lp_->global_box_min_boundary_.z() ||
      !local_data_.curr_pos_.allFinite() ||
      local_data_.curr_pos_.z() > hard_ceiling_z + 1.0e-6) {
    last_plan_fail_reason_ =
        "escape: invalid or already violated hard ceiling";
    ROS_WARN_THROTTLE(
        2.0, "[local-plan] %s (virtual=%.3f, hard=%.3f, z=%.3f)",
        last_plan_fail_reason_.c_str(), virtual_ceiling_z, hard_ceiling_z,
        local_data_.curr_pos_.z());
    return false;
  }

  double time_now = (ros::Time::now() - local_data_.start_time_).toSec();
  Eigen::Vector3d start_pos;
  if (is_static) {
    start_pos = local_data_.curr_pos_;
  } else {
    time_now =
        time_now > local_data_.duration_ ? local_data_.duration_ : time_now;
    start_pos = local_data_.minco_traj_.getPos(time_now);
  }
  if (!start_pos.allFinite()) {
    last_plan_fail_reason_ = "escape: invalid current start position";
    return false;
  }

  Eigen::Vector3f min_bd, max_bd;
  for (int i = 0; i < 3; i++) {
    min_bd[i] = static_cast<float>(start_pos[i] - 2.0);
    max_bd[i] = static_cast<float>(start_pos[i] + 2.0);
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
  if (surf_points.empty()) {
    last_plan_fail_reason_ = "escape: no local obstacle samples for FIRI";
    ROS_WARN_THROTTLE(2.0, "[local-plan] %s",
                      last_plan_fail_reason_.c_str());
    return false;
  }
  Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
  bd(0, 0) = 1.0;
  bd(1, 0) = -1.0;
  bd(2, 1) = 1.0;
  bd(3, 1) = -1.0;
  bd(4, 2) = 1.0;
  bd(5, 2) = -1.0;
  bd(0, 3) =
      -std::min(start_pos.x() + 2.0,
                static_cast<double>(
                    lidar_map_interface_->lp_->global_map_max_boundary_[0]));
  bd(1, 3) = std::max(start_pos.x() - 2.0,
                      static_cast<double>(
                          lidar_map_interface_->lp_->global_box_min_boundary_[0]));
  bd(2, 3) =
      -std::min(start_pos.y() + 2.0,
                static_cast<double>(
                    lidar_map_interface_->lp_->global_map_max_boundary_[1]));
  bd(3, 3) = std::max(start_pos.y() - 2.0,
                      static_cast<double>(
                          lidar_map_interface_->lp_->global_box_min_boundary_[1]));
  bd(4, 3) =
      -std::min(start_pos.z() + 1.0,
                std::min(static_cast<double>(
                             lidar_map_interface_->lp_
                                 ->global_map_max_boundary_[2]),
                         std::max(virtual_ceiling_z, start_pos.z())));
  bd(5, 3) = std::max(start_pos.z() - 1.0,
                      static_cast<double>(
                          lidar_map_interface_->lp_->global_box_min_boundary_[2]));
  Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pc(
      surf_points[0].data(), 3, surf_points.size());

  // Use the exact recovery start as FIRI's seed.  In CAUTION `is_static` is
  // always true, so this is actual odometry rather than topo odom_node.
  const Eigen::Vector3d firi_seed = start_pos;
  Eigen::MatrixX4d hp;
  if (!firi::firi(bd, pc, firi_seed, firi_seed, hp, 2, 1.0e-6,
                  gcopter_config_->firiObstacleDistanceLimit,
                  gcopter_config_->firiMaxPlaneCount) ||
      hp.rows() <= 0) {
    last_plan_fail_reason_ = "escape: base FIRI failed";
    ROS_WARN_THROTTLE(2.0, "[local-plan] %s",
                      last_plan_fail_reason_.c_str());
    return false;
  }
  Eigen::Vector3d inner;
  if (!geo_utils::findInterior(hp, inner)) {
    last_plan_fail_reason_ = "escape: base FIRI has no interior";
    ROS_WARN_THROTTLE(2.0, "[local-plan] %s",
                      last_plan_fail_reason_.c_str());
    return false;
  }

  // --- Keep the CAUTION escape bubble inside the observed region. -----------
  // flyToSafeRegion used to run FIRI against a bare +-2 m / +-1 m box, so the
  // escape polytope could grow into space the sensor has never seen. This
  // applies every active observed-boundary face that contains both the actual
  // escape start and FIRI's seed. The old bubble interior is deliberately NOT
  // a gate: if it lies on the unobserved side, clipping it away and solving for
  // a new interior is exactly the desired behaviour. If start/seed containment
  // or clipped FIRI fails, return failure and remain in CAUTION; never fall back
  // to the unobserved base bubble. Deliberately NOT
  // clipCorridorToObservedBoundary(): that one is driven by a guide path, which
  // an escape does not have.
  if (!clip_escape_to_observed_ || !frontier_manager_) {
    last_plan_fail_reason_ =
        "escape: strict observed-region clipping unavailable";
    ROS_WARN_THROTTLE(2.0, "[local-plan] %s",
                      last_plan_fail_reason_.c_str());
    return false;
  }
  Eigen::MatrixX4d escape_observed_halfspaces(0, 4);
  {
    constexpr double kEps = 1.0e-6;
    ObservedBoundary escape_boundary = buildObservedBoundary();
    // Normal exploration activates a cone face only when frontier evidence
    // proves that it is a current observation boundary. CAUTION cannot use
    // that permissive gate: immediately after takeoff fov_edge_cells_ may be
    // empty, which previously produced zero clipping faces and selected an
    // escape point behind the limited-FOV sensor (09-06-08: first two body-x
    // displacements were -0.399 m and -0.390 m). An emergency positional
    // escape must remain inside what the sensor can see now, even if older map
    // data exists elsewhere, so all four current frustum faces are mandatory.
    for (ObservedBoundaryFace &face : escape_boundary.faces)
      face.active = true;
    publishObservedBoundaryDebug(escape_boundary);
    const Eigen::MatrixX4d active = activeObservedHalfspaces(escape_boundary);
    escape_observed_halfspaces = active;
    const Eigen::Vector4d start_h(start_pos.x(), start_pos.y(), start_pos.z(),
                                  1.0);
    const Eigen::Vector4d seed_h(firi_seed.x(), firi_seed.y(), firi_seed.z(),
                                 1.0);
    for (int row = 0; row < active.rows(); ++row) {
      if ((active.row(row) * start_h)(0) > kEps ||
          (active.row(row) * seed_h)(0) > kEps) {
        last_plan_fail_reason_ =
            "escape: start/seed outside observed boundary";
        ROS_WARN_THROTTLE(2.0,
                          "[local-plan] %s (face %d/%d)",
                          last_plan_fail_reason_.c_str(), row + 1,
                          static_cast<int>(active.rows()));
        return false;
      }
    }
    if (active.rows() > 0) {
      Eigen::MatrixX4d escape_bd(6 + active.rows(), 4);
      escape_bd.topRows<6>() = bd;
      escape_bd.bottomRows(active.rows()) = active;
      // The cone faces pass exactly through the current pose, so the seed sits
      // ON them. firi() tests seed containment with an exact `> 0.0`
      // (firi.hpp:267), which floating-point rounding alone can fail; relax the
      // added rows by the geometric epsilon while solving, as convexCover does
      // (sfc_gen.hpp:167-174). 1e-6 m is far below any physical margin.
      Eigen::MatrixX4d escape_bd_relaxed = escape_bd;
      for (int row = 6; row < escape_bd_relaxed.rows(); ++row)
        escape_bd_relaxed(row, 3) -=
            kEps * escape_bd_relaxed.row(row).head<3>().norm();
      Eigen::MatrixX4d clipped_hp;
      Eigen::Vector3d clipped_inner;
      if (firi::firi(escape_bd_relaxed, pc, firi_seed, firi_seed, clipped_hp, 2,
                     kEps, gcopter_config_->firiObstacleDistanceLimit,
                     gcopter_config_->firiMaxPlaneCount) &&
          clipped_hp.rows() > 0 &&
          geo_utils::findInterior(clipped_hp, clipped_inner) &&
          (clipped_hp * start_h).maxCoeff() <= kEps) {
        hp.swap(clipped_hp);
        inner = clipped_inner;
      } else {
        last_plan_fail_reason_ =
            "escape: strict observed-region FIRI/interior failed";
        ROS_WARN_THROTTLE(2.0,
                          "[local-plan] %s (%d active face(s))",
                          last_plan_fail_reason_.c_str(),
                          static_cast<int>(active.rows()));
        return false;
      }
    }
    if ((hp * start_h).maxCoeff() > kEps) {
      last_plan_fail_reason_ =
          "escape: actual start outside strict escape polytope";
      ROS_WARN_THROTTLE(2.0, "[local-plan] %s",
                        last_plan_fail_reason_.c_str());
      return false;
    }
  }

  // A FIRI plane is a separating plane, not necessarily the physical obstacle
  // surface.  Contracting every such plane by DilateRadiusSoft can therefore
  // erase a corridor even when the measured map contains a valid safe point.
  // Use FIRI only as the strict convex/observed free-space certificate, and use
  // the map's nearest-obstacle distance (the same quantity used by the runtime
  // collision checker) for the actual vehicle clearance.
  const Eigen::MatrixX4d escape_raw_hp = hp;
  const Eigen::MatrixX4d escape_safe_hp = escape_raw_hp;
  constexpr double kTargetGridStep = 0.2; // same resolution as escape cloud
  constexpr double kTargetLineStep = 0.05;
  constexpr double kTargetClearanceTolerance = 1.0e-3;
  const double start_clearance = lidar_map_interface_->getDisToOcc(start_pos);
  Eigen::Vector3d safe_target = Eigen::Vector3d::Zero();
  double safe_target_clearance = -1.0;
  double safe_target_distance = std::numeric_limits<double>::infinity();
  if (std::isfinite(start_clearance) && start_clearance >= 0.0) {
    for (int ix = -10; ix <= 10; ++ix) {
      for (int iy = -10; iy <= 10; ++iy) {
        for (int iz = -5; iz <= 5; ++iz) {
          const Eigen::Vector3d candidate =
              start_pos +
              Eigen::Vector3d(ix * kTargetGridStep, iy * kTargetGridStep,
                              iz * kTargetGridStep);
          const Eigen::Vector4d candidate_h(
              candidate.x(), candidate.y(), candidate.z(), 1.0);
          if (candidate.z() > virtual_ceiling_z + 1.0e-6 ||
              (escape_raw_hp * candidate_h).maxCoeff() > 1.0e-6 ||
              (escape_observed_halfspaces.rows() > 0 &&
               (escape_observed_halfspaces * candidate_h).maxCoeff() >
                   1.0e-6))
            continue;
          const double candidate_clearance =
              lidar_map_interface_->getDisToOcc(candidate);
          if (!std::isfinite(candidate_clearance) ||
              candidate_clearance + kTargetClearanceTolerance <
                  gcopter_config_->dilateRadiusSoft)
            continue;

          // A safe endpoint is insufficient if reaching it first moves closer
          // to an obstacle.  Because the raw FIRI is convex, the straight
          // segment stays inside it; verify that the measured clearance along
          // that segment never drops below the clearance already present at
          // the current pose.
          const Eigen::Vector3d delta = candidate - start_pos;
          const double distance = delta.norm();
          if (!std::isfinite(distance) ||
              distance <= gcopter_config_->vectorNormEps)
            continue;
          const int line_samples =
              std::max(1, static_cast<int>(std::ceil(distance /
                                                     kTargetLineStep)));
          double line_clearance_floor = start_clearance;
          bool non_worsening_line = true;
          for (int sample = 1; sample <= line_samples; ++sample) {
            const Eigen::Vector3d point =
                start_pos + delta *
                                (static_cast<double>(sample) / line_samples);
            const double clearance = lidar_map_interface_->getDisToOcc(point);
            if (!std::isfinite(clearance) ||
                clearance + kTargetClearanceTolerance <
                    line_clearance_floor) {
              non_worsening_line = false;
              break;
            }
            line_clearance_floor =
                std::max(line_clearance_floor, clearance);
          }
          if (!non_worsening_line)
            continue;

          // Prefer the shortest certified egress; clearance breaks ties.
          if (distance + 1.0e-6 < safe_target_distance ||
              (std::abs(distance - safe_target_distance) <= 1.0e-6 &&
               candidate_clearance > safe_target_clearance)) {
            safe_target = candidate;
            safe_target_distance = distance;
            safe_target_clearance = candidate_clearance;
          }
        }
      }
    }
  }
  if (!safe_target.allFinite() || !std::isfinite(safe_target_clearance) ||
      safe_target_clearance < 0.0) {
    last_plan_fail_reason_ =
        "escape: no observed non-worsening path to DilateRadiusSoft";
    ROS_WARN_THROTTLE(2.0, "[local-plan] %s (start clearance=%.3f)",
                      last_plan_fail_reason_.c_str(), start_clearance);
    return false;
  }
  inner = safe_target;

  std::vector<Eigen::MatrixX4d> hPolys;
  hPolys.push_back(escape_raw_hp);
  hPolys.push_back(escape_safe_hp);
  Eigen::Vector4d bh;
  const Eigen::Vector3d inner_delta = inner - firi_seed;
  const double inner_delta_norm = inner_delta.norm();
  Eigen::Vector3d dir = Eigen::Vector3d::Zero();
  if (inner_delta.allFinite() &&
      inner_delta_norm > gcopter_config_->vectorNormEps) {
    dir = inner_delta / inner_delta_norm;
  }

  Eigen::Matrix3d iniState;
  iniState << start_pos, dir * 0.2, Eigen::Vector3d::Zero();
  Eigen::Matrix3d finState;
  ros::Time hpoly_gen_end = ros::Time::now();
  // iniState << topo_graph_->odom_node_->center_, local_data_.curr_vel_,
  // Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
  finState << inner, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
  bh << iniState.topLeftCorner<3, 1>(), 1.0;
  int start_idx = -1;
  for (int i = hPolys.size() - 1; i >= 0; i--) {
    const Eigen::MatrixX4d &candidate_hp = hPolys[i];
    // The observed cone can pass exactly through the current pose.  The
    // strict observed-FIRI check above already accepts that geometric boundary
    // with +1e-6 tolerance; demanding every face be at least 1e-6 *inside*
    // here contradicted that check and rejected every such recovery as
    // "start not in local free bubble".  Containment is sufficient for the
    // raw egress phase; the completed trajectory is independently postchecked.
    if ((candidate_hp * bh).maxCoeff() <= 1.0e-6) {
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
    last_plan_fail_reason_ = std::string("escape: gcopter setup failed: ") +
                             gcopter.lastSetupError();
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
  const double escape_duration =
      local_data_.minco_traj_.getTotalDuration();
  if (!std::isfinite(escape_duration) ||
      escape_duration < gcopter_config_->minSegmentTime) {
    local_data_ = local_data_backup;
    last_plan_fail_reason_ = "escape: invalid trajectory duration";
    return false;
  }
  const Eigen::VectorXd escape_piece_durations =
      local_data_.minco_traj_.getDurations();
  if (escape_piece_durations.size() < 2 ||
      !escape_piece_durations.allFinite() ||
      (escape_piece_durations.array() <= 0.0).any()) {
    local_data_ = local_data_backup;
    last_plan_fail_reason_ =
        "escape: trajectory does not preserve raw-to-safe corridor phases";
    return false;
  }

  // Optimizer constraints are soft/numerical.  Before publishing a CAUTION
  // recovery, independently sample the finished trajectory against the exact
  // observed half-spaces, raw/safe FIRI phases, measured obstacle clearance,
  // and the virtual ceiling.  Before the safe phase, the only exception is the
  // clearance already present at t=0: clearance may never decrease.  Once the
  // soft margin is reached it may never be surrendered.
  constexpr double kEscapeSampleStep = 0.02;
  constexpr double kEscapeBoundaryTolerance = 1.0e-5;
  constexpr double kEscapeClearanceTolerance = 1.0e-3;
  const double escape_start_clearance =
      lidar_map_interface_->getDisToOcc(start_pos);
  if (!std::isfinite(escape_start_clearance) ||
      escape_start_clearance < 0.0) {
    local_data_ = local_data_backup;
    last_plan_fail_reason_ = "escape: invalid start clearance";
    return false;
  }
  double clearance_floor = escape_start_clearance;
  bool reached_soft_clearance =
      escape_start_clearance + kEscapeClearanceTolerance >=
      gcopter_config_->dilateRadiusSoft;
  double escape_soft_entry_time =
      reached_soft_clearance ? 0.0
                             : std::numeric_limits<double>::quiet_NaN();
  double minimum_clearance = escape_start_clearance;
  double ceiling_floor = start_pos.z();
  bool reached_virtual_ceiling =
      start_pos.z() <= virtual_ceiling_z + kEscapeBoundaryTolerance;
  for (double sample_time = 0.0;; sample_time += kEscapeSampleStep) {
    const double t = std::min(sample_time, escape_duration);
    const Eigen::Vector3d position = local_data_.minco_traj_.getPos(t);
    const Eigen::Vector4d position_h(position.x(), position.y(), position.z(),
                                     1.0);
    if (!position.allFinite() ||
        position.z() > hard_ceiling_z + kEscapeBoundaryTolerance) {
      local_data_ = local_data_backup;
      last_plan_fail_reason_ =
          "escape: post-check exceeds hard ceiling";
      ROS_WARN_THROTTLE(2.0, "[local-plan] %s at t=%.3f",
                        last_plan_fail_reason_.c_str(), t);
      return false;
    }
    if (!reached_virtual_ceiling) {
      if (position.z() > ceiling_floor + kEscapeBoundaryTolerance) {
        local_data_ = local_data_backup;
        last_plan_fail_reason_ =
            "escape: post-check climbs during ceiling recovery";
        ROS_WARN_THROTTLE(2.0, "[local-plan] %s at t=%.3f",
                          last_plan_fail_reason_.c_str(), t);
        return false;
      }
      ceiling_floor = std::min(ceiling_floor, position.z());
      reached_virtual_ceiling =
          position.z() <= virtual_ceiling_z + kEscapeBoundaryTolerance;
    } else if (position.z() >
               virtual_ceiling_z + kEscapeBoundaryTolerance) {
      local_data_ = local_data_backup;
      last_plan_fail_reason_ =
          "escape: post-check leaves virtual ceiling";
      ROS_WARN_THROTTLE(2.0, "[local-plan] %s at t=%.3f",
                        last_plan_fail_reason_.c_str(), t);
      return false;
    }
    if ((escape_raw_hp * position_h).maxCoeff() >
        kEscapeBoundaryTolerance) {
      local_data_ = local_data_backup;
      last_plan_fail_reason_ = "escape: post-check leaves raw FIRI polytope";
      ROS_WARN_THROTTLE(2.0, "[local-plan] %s at t=%.3f",
                        last_plan_fail_reason_.c_str(), t);
      return false;
    }
    if (escape_observed_halfspaces.rows() > 0 &&
        (escape_observed_halfspaces * position_h).maxCoeff() >
            kEscapeBoundaryTolerance) {
      local_data_ = local_data_backup;
      last_plan_fail_reason_ =
          "escape: post-check leaves observed region";
      ROS_WARN_THROTTLE(2.0, "[local-plan] %s at t=%.3f",
                        last_plan_fail_reason_.c_str(), t);
      return false;
    }
    const double clearance = lidar_map_interface_->getDisToOcc(position);
    if (!std::isfinite(clearance) || clearance < 0.0) {
      local_data_ = local_data_backup;
      last_plan_fail_reason_ = "escape: post-check invalid obstacle clearance";
      return false;
    }
    minimum_clearance = std::min(minimum_clearance, clearance);
    if (!reached_soft_clearance) {
      if (clearance + kEscapeClearanceTolerance < clearance_floor) {
        local_data_ = local_data_backup;
        last_plan_fail_reason_ =
            "escape: post-check worsens clearance before safe region";
        ROS_WARN_THROTTLE(
            2.0, "[local-plan] %s at t=%.3f (%.3f < %.3f)",
            last_plan_fail_reason_.c_str(), t, clearance, clearance_floor);
        return false;
      }
      clearance_floor = std::max(clearance_floor, clearance);
      reached_soft_clearance =
          clearance + kEscapeClearanceTolerance >=
          gcopter_config_->dilateRadiusSoft;
      if (reached_soft_clearance)
        escape_soft_entry_time = t;
    } else if (clearance + kEscapeClearanceTolerance <
               gcopter_config_->dilateRadiusSoft) {
      local_data_ = local_data_backup;
      last_plan_fail_reason_ =
          "escape: post-check loses soft clearance after reaching it";
      ROS_WARN_THROTTLE(2.0, "[local-plan] %s at t=%.3f",
                        last_plan_fail_reason_.c_str(), t);
      return false;
    }
    if (t >= escape_duration)
      break;
  }
  if (!reached_soft_clearance || !std::isfinite(escape_soft_entry_time)) {
    local_data_ = local_data_backup;
    last_plan_fail_reason_ =
        "escape: post-check never reaches DilateRadiusSoft";
    return false;
  }
  if (!reached_virtual_ceiling) {
    local_data_ = local_data_backup;
    last_plan_fail_reason_ =
        "escape: post-check never re-enters virtual ceiling";
    return false;
  }
  if (local_data_.minco_traj_.getPieceNum() > 0) {
    // ROS_INFO_STREAM(
    // "local_data_.minco_traj_.getPieceNum(): " <<
    // local_data_.minco_traj_.getPieceNum());
    // hpoly_gen_end is what local_data_.start_time_ is set to below. The
    // escape path runs no new YawTrajOpt, but the carried-over minco_yaw_traj_
    // is what fast_exploration_fsm re-publishes against the new start_time_,
    // so it is the yaw the drone will actually track -- draw the frustums
    // from it.
    gcopter_viz_->visualize(local_data_.minco_traj_,
                            local_data_.minco_yaw_traj_, hpoly_gen_end,
                            gcopter_config_->maxVelMag);
  } else {
    local_data_ = local_data_backup;
    last_plan_fail_reason_ = "escape: optimized traj empty";
    ROS_WARN_THROTTLE(2.0, "[local-plan] %s", last_plan_fail_reason_.c_str());
    return false;
  }
  local_data_.traj_id_ += 1;
  local_data_.start_time_ = hpoly_gen_end;
  local_data_.start_pos_ = start_pos;
  local_data_.duration_ = escape_duration;
  // 성공: 단 이것은 요청 경로가 아니라 "안전지대 탈출" 궤적임을 표시
  local_data_.rotate_in_place_ = false;
  local_data_.caution_escape_ = true;
  local_data_.escape_soft_entry_time_ = escape_soft_entry_time;
  local_data_.escape_virtual_ceiling_z_ = virtual_ceiling_z;
  local_data_.escape_hard_ceiling_z_ = hard_ceiling_z;
  local_data_.escape_raw_polytope_ = escape_raw_hp;
  local_data_.escape_safe_polytope_ = escape_safe_hp;
  local_data_.escape_observed_halfspaces_ = escape_observed_halfspaces;
  last_plan_fail_reason_.clear();
  last_plan_was_escape_ = true;
  ROS_INFO("[CAUTION] accepted escape: duration=%.2f s, soft-entry=%.2f s, "
           "clearance %.3f -> %.3f (required %.3f, min %.3f)",
           escape_duration, escape_soft_entry_time, escape_start_clearance,
           safe_target_clearance, gcopter_config_->dilateRadiusSoft,
           minimum_clearance);
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
