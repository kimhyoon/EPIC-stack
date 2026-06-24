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
#include <lidar_map/lidar_map.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
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

  // Check if transform is needed
  if (cloud_frame_ == map_frame_) {
    needs_transform_ = false;
    ROS_INFO("[LIOInterface] Cloud frame '%s' matches map frame '%s', no transform needed",
             cloud_frame_.c_str(), map_frame_.c_str());
    return;
  }

  // Lookup static transform: body_frame -> cloud_frame
  try {
    geometry_msgs::TransformStamped transform_stamped =
        tf_buffer_->lookupTransform(body_frame_, cloud_frame_, ros::Time(0), ros::Duration(5.0));

    // Convert to Eigen::Matrix4f
    Eigen::Isometry3d transform_eigen = tf2::transformToEigen(transform_stamped.transform);
    T_body_to_cloud_ = transform_eigen.matrix().cast<float>();

    needs_transform_ = true;
    ROS_INFO("[LIOInterface] Successfully looked up transform %s -> %s",
             body_frame_.c_str(), cloud_frame_.c_str());
    ROS_INFO("[LIOInterface] Transform will be applied: map(%s) -> body(%s) -> cloud(%s)",
             map_frame_.c_str(), body_frame_.c_str(), cloud_frame_.c_str());
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("[LIOInterface] Failed to lookup transform %s -> %s: %s",
              body_frame_.c_str(), cloud_frame_.c_str(), ex.what());
    ROS_ERROR("[LIOInterface] Proceeding without transform - pointcloud may be misaligned!");
    needs_transform_ = false;
  }
}

double LIOInterface::getDisToOcc(const PointType &pt) {
  PointVector nes_pts;
  vector<float> diss;
  KNN(pt, 1, nes_pts, diss);
  if (nes_pts.size() == 0)
    return 10.0;
  else
    return sqrt(diss[0]);
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
  ikd_Tree_map.Nearest_Search(pt, k, pts, dis, 10.0);
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
void LIOInterface::updateCloudMapOdometry(
    const sensor_msgs::PointCloud2ConstPtr &msg,
    const nav_msgs::Odometry::ConstPtr &odom_) {
  ld_->map_update = true;
  static Eigen::Vector3f last_lidar_pose(0, 0, 0);
  Eigen::Vector3f lidar_pos_(odom_->pose.pose.position.x,
                             odom_->pose.pose.position.y,
                             odom_->pose.pose.position.z);
  Eigen::Vector3f lidar_vel_(odom_->twist.twist.linear.x,
                             odom_->twist.twist.linear.y,
                             odom_->twist.twist.linear.z);
  last_lidar_pose = lidar_pos_;
  ld_->lidar_pose_ = lidar_pos_;
  ld_->lidar_vel_ = lidar_vel_;
  Eigen::AngleAxisf y_axis_angle(M_PI / 180.0 * lp_->lidar_pitch_,
                                 Eigen::Vector3f::UnitY());
  Eigen::Quaternionf q_y(y_axis_angle);
  ld_->lidar_q_ = Eigen::Quaternionf(odom_->pose.pose.orientation.w,
                                     odom_->pose.pose.orientation.x,
                                     odom_->pose.pose.orientation.y,
                                     odom_->pose.pose.orientation.z) *
                  q_y;

  pcl::PointCloud<pcl::PointXYZ> cloud_input;
  pcl::fromROSMsg(*msg, cloud_input);

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
    return;
  }

  // Apply frame transformation if needed (on filtered cloud for efficiency)
  // ros::Time t_transform_start = ros::Time::now();
  if (needs_transform_) {
    // ROS_INFO_THROTTLE(5.0, "[LIOInterface] Applying frame transformation to %lu filtered points",
    //                   filtered_points->points.size());
    // Build T_map_body from odometry
    Eigen::Matrix4f T_map_body = Eigen::Matrix4f::Identity();
    T_map_body.block<3, 1>(0, 3) = lidar_pos_;
    Eigen::Quaternionf q_map_body(odom_->pose.pose.orientation.w,
                                   odom_->pose.pose.orientation.x,
                                   odom_->pose.pose.orientation.y,
                                   odom_->pose.pose.orientation.z);
    T_map_body.block<3, 3>(0, 0) = q_map_body.toRotationMatrix();

    // Compute T_map_cloud = T_map_body * T_body_cloud
    Eigen::Matrix4f T_map_cloud = T_map_body * T_body_to_cloud_;

    // Transform pointcloud (in-place)
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*filtered_points, *transformed_cloud, T_map_cloud);
    filtered_points = transformed_cloud;
    // ROS_INFO_THROTTLE(5.0, "[LIOInterface] Transform completed (took %.2fms)",
    //                   (ros::Time::now() - t_transform_start).toSec() * 1000.0);
  }

  // BUGFIX: Store the processed cloud in ld_->lidar_cloud_ (was missing in frame transform version)
  ld_->lidar_cloud_ = *filtered_points;
  PointVector pcl_map = filtered_points->points;

  if (ld_->first_map_flag_) {
    // this->ikd_Tree_map(0.3,0.6,0.2);
    this->ikd_Tree_map.set_downsample_param(0.1);
    this->ikd_Tree_map.Build(pcl_map);
    ld_->first_map_flag_ = false;
  } else {
    this->ikd_Tree_map.Add_Points(pcl_map, true);
  }
  ros::Time ikd_update_end_stamp = ros::Time::now();
}

} // namespace fast_planner