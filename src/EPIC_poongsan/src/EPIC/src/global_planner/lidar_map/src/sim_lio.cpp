/*
If you need to replace with other point cloud data structures, please
re-implement the following interfaces:

1. getDisToOcc: Returns the distance from the specified point to the nearest
obstacle in the map.
2. KNN: Nearest neighbor search. boxSearch: Region search.
3. updateCloudMapOdometry: Point cloud map update. 
4. LIOInterfaceData::Ptr ld: Current frame world coordinate point clouds and
lidar-odometry.

If you need to integrate EPIC with Lidar SLAM algorithm and shares
memory, thread mutual exclusion should be noted.
*/
#include "visualization_msgs/Marker.h"
#include <algorithm>
#include <cmath>
#include <pcl/filters/filter.h>
#include <lidar_map/lidar_map.h>
#include <lidar_map/structured_log.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/TransformStamped.h>
namespace fast_planner {

void LIOInterface::initializeTransform(const std::string& map_frame,
                                       const std::string& body_frame,
                                       const std::string& cloud_frame) {
  map_frame_ = map_frame;
  body_frame_ = body_frame;
  cloud_frame_ = cloud_frame;

  // Check if transform is needed.
  if (cloud_frame_ == map_frame_) {
    needs_transform_ = false;
    EPIC_SENSOR_INFO("sensor.lidar", "cloud frame=%s matches map frame=%s; transform disabled",
             cloud_frame_.c_str(), map_frame_.c_str());
    return;
  }

  // Lookup static transform: body_frame -> cloud_frame.
  try {
    geometry_msgs::TransformStamped transform_stamped =
        tf_buffer_->lookupTransform(body_frame_, cloud_frame_, ros::Time(0), ros::Duration(5.0));

    // Convert to Eigen::Matrix4f.
    Eigen::Isometry3d transform_eigen = tf2::transformToEigen(transform_stamped.transform);
    T_body_to_cloud_ = transform_eigen.matrix().cast<float>();

    needs_transform_ = true;
    EPIC_SENSOR_INFO("sensor.lidar", "transform ready map=%s body=%s cloud=%s",
             map_frame_.c_str(), body_frame_.c_str(), cloud_frame_.c_str());
  } catch (tf2::TransformException &ex) {
    EPIC_SENSOR_ERROR("sensor.lidar", "failed transform lookup %s -> %s: %s",
              body_frame_.c_str(), cloud_frame_.c_str(), ex.what());
    EPIC_SENSOR_ERROR("sensor.lidar", "transform disabled; pointcloud may be misaligned");
    needs_transform_ = false;
  }
}

double LIOInterface::getDisToOcc(const PointType &pt) {
  PointVector nes_pts;
  vector<float> diss;
  KNN(pt, 1, nes_pts, diss);
  if (nes_pts.empty() || diss.empty() || !std::isfinite(diss[0]) ||
      diss[0] < 0.0)
    return lp_->max_ray_length_;
  return std::sqrt(diss[0]);
}
double LIOInterface::getDisToOcc(const Eigen::Vector3d &pt) {
  PointType p;
  p.x = pt.x();
  p.y = pt.y();
  p.z = pt.z();
  return getDisToOcc(p);
}
double LIOInterface::getDisToOcc(const Eigen::Vector3f &pt) {
  PointType p;
  p.x = pt.x();
  p.y = pt.y();
  p.z = pt.z();
  return getDisToOcc(p);
}
void LIOInterface::KNN(const PointType &pt, int k, PointVector &pts,
                       vector<float> &dis) {
  ikd_Tree_map.Nearest_Search(pt, k, pts, dis, lp_->max_ray_length_);
}
void LIOInterface::boxSearch(const Eigen::Vector3f &min_bd,
                             const Eigen::Vector3f &max_bd, PointVector &pts) {
  BoxPointType boxpoint;
  for (int i = 0; i < 3; i++) {
    boxpoint.vertex_min[i] = min_bd(i);
    boxpoint.vertex_max[i] = max_bd(i);
  }
  ikd_Tree_map.Box_Search(boxpoint, pts);
}

int LIOInterface::clearFreeSpaceAlongRays(
    const Eigen::Vector3f &sensor_origin,
    const pcl::PointCloud<pcl::PointXYZ> &current_world_cloud) {
  if (!lp_->ray_clearing_enabled_ || current_world_cloud.empty() ||
      ld_->first_map_flag_) {
    return 0;
  }

  // Index only the directions of this scan.  This is temporary per-frame
  // visibility data, not a second occupancy map.  For each existing map point
  // we find the closest actual measured ray, then delete the point only when it
  // lies inside that ray's narrow free-space tube and before the protected
  // endpoint band.
  pcl::PointCloud<pcl::PointXYZ>::Ptr ray_directions(
      new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<float> ray_ranges;
  ray_directions->points.reserve(current_world_cloud.points.size());
  ray_ranges.reserve(current_world_cloud.points.size());

  float max_observed_range = 0.0f;
  for (const auto &endpoint : current_world_cloud.points) {
    const Eigen::Vector3f ray(endpoint.x - sensor_origin.x(),
                              endpoint.y - sensor_origin.y(),
                              endpoint.z - sensor_origin.z());
    const float range = ray.norm();
    if (!std::isfinite(range) || range <= lp_->min_ray_length_ ||
        range > lp_->max_ray_length_) {
      continue;
    }
    const Eigen::Vector3f direction = ray / range;
    ray_directions->points.emplace_back(direction.x(), direction.y(),
                                        direction.z());
    ray_ranges.push_back(range);
    max_observed_range = std::max(max_observed_range, range);
  }
  if (ray_directions->points.empty()) {
    return 0;
  }
  ray_directions->width =
      static_cast<uint32_t>(ray_directions->points.size());
  ray_directions->height = 1;
  ray_directions->is_dense = true;

  pcl::KdTreeFLANN<pcl::PointXYZ> direction_tree;
  direction_tree.setInputCloud(ray_directions);

  BoxPointType query_box;
  for (int axis = 0; axis < 3; ++axis) {
    query_box.vertex_min[axis] = sensor_origin(axis) - max_observed_range;
    query_box.vertex_max[axis] = sensor_origin(axis) + max_observed_range;
  }
  PointVector candidates;
  ikd_Tree_map.Box_Search(query_box, candidates);

  const float clear_radius_sq = static_cast<float>(
      lp_->ray_clearing_radius_ * lp_->ray_clearing_radius_);
  PointVector points_to_delete;
  points_to_delete.reserve(candidates.size() / 8);
  std::vector<int> nearest_index(1);
  std::vector<float> nearest_distance_sq(1);

  for (const auto &map_point : candidates) {
    const Eigen::Vector3f relative(map_point.x - sensor_origin.x(),
                                   map_point.y - sensor_origin.y(),
                                   map_point.z - sensor_origin.z());
    const float map_range = relative.norm();
    if (!std::isfinite(map_range) || map_range <= lp_->vector_norm_eps_ ||
        map_range > max_observed_range) {
      continue;
    }

    const Eigen::Vector3f map_direction = relative / map_range;
    pcl::PointXYZ direction_query(map_direction.x(), map_direction.y(),
                                  map_direction.z());
    if (direction_tree.nearestKSearch(direction_query, 1, nearest_index,
                                      nearest_distance_sq) != 1) {
      continue;
    }

    const int ray_index = nearest_index[0];
    const auto &ray_point = ray_directions->points[ray_index];
    const Eigen::Vector3f ray_direction(ray_point.x, ray_point.y, ray_point.z);
    const float projection = relative.dot(ray_direction);
    // min_ray_length_ validates measured endpoints; it must not protect stale
    // map points near the sensor.  A valid return certifies free space from the
    // sensor origin all the way to its protected endpoint band.
    if (projection <= lp_->vector_norm_eps_ ||
        projection >= ray_ranges[ray_index] - lp_->ray_endpoint_margin_) {
      continue;
    }

    const float lateral_distance_sq =
        std::max(0.0f, relative.squaredNorm() - projection * projection);
    if (lateral_distance_sq <= clear_radius_sq) {
      points_to_delete.push_back(map_point);
    }
  }

  if (points_to_delete.empty()) {
    return 0;
  }
  ikd_Tree_map.Delete_Points(points_to_delete);
  ROS_DEBUG_THROTTLE(
      1.0,
      "[LIOInterface] ray clearing removed %zu/%zu local map points using "
      "%zu valid rays",
      points_to_delete.size(), candidates.size(), ray_ranges.size());
  return static_cast<int>(points_to_delete.size());
}

bool LIOInterface::updateCloudMapOdometry(
    const sensor_msgs::PointCloud2ConstPtr &msg,
    const nav_msgs::Odometry::ConstPtr &odom_) {
  static Eigen::Vector3f last_lidar_pose(0, 0, 0);
  Eigen::Vector3f lidar_pos_(odom_->pose.pose.position.x,
                             odom_->pose.pose.position.y,
                             odom_->pose.pose.position.z);
  Eigen::Vector3f lidar_vel_(odom_->twist.twist.linear.x,
                             odom_->twist.twist.linear.y,
                             odom_->twist.twist.linear.z);
  Eigen::Quaternionf q_map_body(odom_->pose.pose.orientation.w,
                                odom_->pose.pose.orientation.x,
                                odom_->pose.pose.orientation.y,
                                odom_->pose.pose.orientation.z);
  if (!lidar_pos_.allFinite() || !lidar_vel_.allFinite() ||
      !q_map_body.coeffs().allFinite() ||
      q_map_body.squaredNorm() <=
          lp_->vector_norm_eps_ * lp_->vector_norm_eps_) {
    EPIC_SENSOR_WARN_THROTTLE(
        1.0, "sensor.lidar", "ignoring synchronized cloud/odometry pair with invalid pose");
    return false;
  }
  q_map_body.normalize();
  // Sensor mounting rotation for lidar_q_ visualization/FOV logic. If odometry
  // is already in the lidar frame, both values should be zero.
  Eigen::AngleAxisf y_axis_angle(M_PI / 180.0 * lp_->lidar_pitch_,
                                 Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf z_axis_angle(M_PI / 180.0 * lp_->lidar_yaw_,
                                 Eigen::Vector3f::UnitZ());
  Eigen::Quaternionf q_y(y_axis_angle);
  Eigen::Quaternionf q_z(z_axis_angle);
  const Eigen::Quaternionf lidar_q = q_map_body * q_z * q_y;

  pcl::PointCloud<pcl::PointXYZ> cloud_input;
  pcl::fromROSMsg(*msg, cloud_input);
  std::vector<int> finite_indices;
  pcl::PointCloud<pcl::PointXYZ> finite_cloud;
  pcl::removeNaNFromPointCloud(cloud_input, finite_cloud, finite_indices);
  cloud_input.swap(finite_cloud);

  // The ML-X /ml_/pointcloud stream contains exact zero placeholders and can
  // also report physically invalid returns just centimetres from the sensor.
  // Reject both before voxelization and, critically, before permanent map
  // insertion. The radial range test is valid only for sensor-frame clouds;
  // world-frame coordinates must never be measured from the world origin.
  size_t zero_point_count = 0;
  size_t near_point_count = 0;
  const double min_range_squared =
      lp_->min_ray_length_ * lp_->min_ray_length_;
  const auto invalid_begin = std::remove_if(
      cloud_input.points.begin(), cloud_input.points.end(),
      [&](const pcl::PointXYZ &point) {
        if (point.x == 0.0f && point.y == 0.0f && point.z == 0.0f) {
          ++zero_point_count;
          return true;
        }
        const double range_squared =
            static_cast<double>(point.x) * point.x +
            static_cast<double>(point.y) * point.y +
            static_cast<double>(point.z) * point.z;
        if (needs_transform_ && range_squared <= min_range_squared) {
          ++near_point_count;
          return true;
        }
        return false;
      });
  cloud_input.points.erase(invalid_begin, cloud_input.points.end());
  cloud_input.width = static_cast<uint32_t>(cloud_input.points.size());
  cloud_input.height = 1;
  if (zero_point_count > 0) {
    EPIC_SENSOR_DEBUG_THROTTLE(5.0, "sensor.lidar",
                       "removed %zu zero-point placeholders",
                       zero_point_count);
  }
  if (near_point_count > 0) {
    EPIC_SENSOR_DEBUG_THROTTLE(
        5.0, "sensor.lidar",
        "rejected %zu sensor returns at/below %.3f m",
        near_point_count, lp_->min_ray_length_);
  }
  if (cloud_input.empty()) {
    EPIC_SENSOR_WARN_THROTTLE(1.0, "sensor.lidar",
                              "pointcloud contains no usable finite points");
    return false;
  }

  // ROS_INFO_THROTTLE(1.0, "[LIOInterface] Input cloud size: %lu points", cloud_input.points.size());

  // OPTIMIZATION: Voxel filter BEFORE transformation (process fewer points)
  ros::Time start = ros::Time::now();
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setLeafSize(0.1, 0.1, 0.1);
  vg.setInputCloud(cloud_input.makeShared());
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points(
      new pcl::PointCloud<pcl::PointXYZ>);
  vg.filter(*filtered_points);

  // ROS_INFO_THROTTLE(1.0, "[LIOInterface] After voxel filter: %lu points (took %.2fms)",
  //                   filtered_points->points.size(),
  //                   (ros::Time::now() - start).toSec() * 1000.0);

  if (filtered_points->points.empty()) {
    // ROS_WARN_THROTTLE(1.0, "[LIOInterface] Point cloud empty after filtering! Input had %lu points",
    //                   cloud_input.points.size());
    return false;
  }

  // Apply frame transformation if needed (on filtered cloud for efficiency).
  // ros::Time t_transform_start = ros::Time::now();
  Eigen::Vector3f sensor_origin = lidar_pos_;
  if (needs_transform_) {
    // ROS_INFO_THROTTLE(5.0, "[LIOInterface] Applying frame transformation to %lu filtered points",
    //                   filtered_points->points.size());
    // Build T_map_body from odometry.
    Eigen::Matrix4f T_map_body = Eigen::Matrix4f::Identity();
    T_map_body.block<3, 1>(0, 3) = lidar_pos_;
    T_map_body.block<3, 3>(0, 0) = q_map_body.toRotationMatrix();

    // Compute T_map_cloud = T_map_body * T_body_cloud.
    Eigen::Matrix4f T_map_cloud = T_map_body * T_body_to_cloud_;
    sensor_origin = T_map_cloud.block<3, 1>(0, 3);

    // Transform pointcloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*filtered_points, *transformed_cloud, T_map_cloud);
    filtered_points = transformed_cloud;
    // ROS_INFO_THROTTLE(5.0, "[LIOInterface] Transform completed (took %.2fms)",
    //                   (ros::Time::now() - t_transform_start).toSec() * 1000.0);
  }

  // Publish exactly the cloud consumed by the planner for frame validation.
  // PointCloud2 conversion runs only while RViz or another tool subscribes.
  if (transformed_cloud_puber_.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2 world_cloud;
    pcl::toROSMsg(*filtered_points, world_cloud);
    world_cloud.header.stamp = msg->header.stamp;
    world_cloud.header.frame_id =
        odom_->header.frame_id.empty() ? "map" : odom_->header.frame_id;
    transformed_cloud_puber_.publish(world_cloud);
  }

  // Store the processed cloud for downstream frontier/viewpoint updates.
  ld_->map_update = true;
  last_lidar_pose = lidar_pos_;
  ld_->lidar_pose_ = lidar_pos_;
  ld_->lidar_vel_ = lidar_vel_;
  ld_->lidar_q_ = lidar_q;
  ld_->lidar_cloud_ = *filtered_points;
  PointVector pcl_map = filtered_points->points;

  if (ld_->first_map_flag_) {
    // this->ikd_Tree_map(0.3,0.6,0.2);
    this->ikd_Tree_map.set_downsample_param(0.1);
    this->ikd_Tree_map.Build(pcl_map);
    ld_->first_map_flag_ = false;
  } else {
    clearFreeSpaceAlongRays(sensor_origin, *filtered_points);
    this->ikd_Tree_map.Add_Points(pcl_map, true);
  }
  ros::Time ikd_update_end_stamp = ros::Time::now();
  return true;
}

} // namespace fast_planner
