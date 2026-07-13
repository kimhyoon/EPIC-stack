#include "visualization_msgs/Marker.h"
#include <cmath>
#include <lidar_map/lidar_map.h>
#include <pcl/filters/voxel_grid.h>
namespace fast_planner {

LIOInterface::LIOInterface() {}

LIOInterface::~LIOInterface() {}

void LIOInterface::init(ros::NodeHandle &nh) {
  lp_.reset(new LIOInterfaceParam);
  ld_.reset(new LIOInterfaceData);
  ikd_Tree_map.setMap_ikdtree(this);
  ikd_Tree_map.Set_delete_criterion_param(0.3);
  ikd_Tree_map.Set_balance_criterion_param(0.6);
  string odom_topic, cloud_topic;
  // Topic names must come from the config yaml. Refuse to start instead of
  // silently running with a stale fallback topic.
  if (!nh.getParam("odometry_topic", odom_topic) || odom_topic.empty() ||
      !nh.getParam("cloud_topic", cloud_topic) || cloud_topic.empty()) {
    ROS_FATAL("[LIOInterface] odometry_topic / cloud_topic not set in config "
              "yaml. REFUSING TO START - no fallback.");
    ros::shutdown();
    exit(1);
  }
  nh.getParam("box_num", lp_->box_num_);
  nh.getParam("lidar_perception/lidar_pitch", lp_->lidar_pitch_);
  // Keep old configs valid when the yaw key is absent.
  nh.param("lidar_perception/lidar_yaw", lp_->lidar_yaw_, 0.0);
  for (int i = 0; i < lp_->box_num_; i++) {
    std::vector<double> tmp;
    nh.getParam("box_" + to_string(i) + "/down", tmp);
    Eigen::Vector3f tmp1(tmp[0], tmp[1], tmp[2]);
    nh.getParam("box_" + to_string(i) + "/up", tmp);
    Eigen::Vector3f tmp2(tmp[0], tmp[1], tmp[2]);
    Eigen::Vector3f min, max;
    for (int i = 0; i < 3; i++) {
      min(i) = fmin(tmp1(i), tmp2(i));
      max(i) = fmax(tmp1(i), tmp2(i));
    }
    lp_->global_box_min_boundary_vec_.emplace_back(min);
    lp_->global_box_max_boundary_vec_.emplace_back(max);
  }
  nh.getParam("dead_area_num", lp_->dead_area_num_);
  for (int i = 0; i < lp_->dead_area_num_; i++) {
    std::vector<double> tmp;
    nh.getParam("dead_" + to_string(i) + "/down", tmp);
    Eigen::Vector3f tmp1(tmp[0], tmp[1], tmp[2]);
    nh.getParam("dead_" + to_string(i) + "/up", tmp);
    Eigen::Vector3f tmp2(tmp[0], tmp[1], tmp[2]);
    Eigen::Vector3f min, max;
    for (int i = 0; i < 3; i++) {
      min(i) = fmin(tmp1(i), tmp2(i));
      max(i) = fmax(tmp1(i), tmp2(i));
    }
    lp_->dead_area_min_boundary_vec_.emplace_back(min);
    lp_->dead_area_max_boundary_vec_.emplace_back(max);
  }

  lp_->global_box_max_boundary_ = lp_->global_box_min_boundary_vec_[0];
  lp_->global_box_min_boundary_ = lp_->global_box_max_boundary_vec_[0];
  for (auto &vec : lp_->global_box_min_boundary_vec_) {
    lp_->global_box_min_boundary_.x() =
        vec.x() < lp_->global_box_min_boundary_.x()
            ? vec.x()
            : lp_->global_box_min_boundary_.x();
    lp_->global_box_min_boundary_.y() =
        vec.y() < lp_->global_box_min_boundary_.y()
            ? vec.y()
            : lp_->global_box_min_boundary_.y();
    lp_->global_box_min_boundary_.z() =
        vec.z() < lp_->global_box_min_boundary_.z()
            ? vec.z()
            : lp_->global_box_min_boundary_.z();
  }
  for (auto &vec : lp_->global_box_max_boundary_vec_) {
    lp_->global_box_max_boundary_.x() =
        vec.x() > lp_->global_box_max_boundary_.x()
            ? vec.x()
            : lp_->global_box_max_boundary_.x();
    lp_->global_box_max_boundary_.y() =
        vec.y() > lp_->global_box_max_boundary_.y()
            ? vec.y()
            : lp_->global_box_max_boundary_.y();
    lp_->global_box_max_boundary_.z() =
        vec.z() > lp_->global_box_max_boundary_.z()
            ? vec.z()
            : lp_->global_box_max_boundary_.z();
  }
  lp_->global_map_max_boundary_ = lp_->global_box_max_boundary_;
  lp_->global_map_min_boundary_ = lp_->global_box_min_boundary_;
  cout << "max boundary: " << lp_->global_map_max_boundary_ << endl;
  cout << "min boundary: " << lp_->global_map_min_boundary_ << endl;
  nh.param("lidar_perception/fov_up", lp_->fov_up, -0.1);
  nh.param("lidar_perception/fov_down", lp_->fov_down, -0.1);
  nh.param("lidar_perception/fov_horizontal", lp_->fov_horizontal, 360.0);
  nh.param("lidar_perception/fov_viewpoint_up", lp_->fov_vp_up, -0.1);
  nh.param("lidar_perception/fov_viewpoint_down", lp_->fov_vp_down, -0.1);
  nh.getParam("lidar_perception/max_ray_length", lp_->max_ray_length_);
  ld_->first_map_flag_ = true;

  // Initialize TF listener for frame transforms.
  tf_buffer_.reset(new tf2_ros::Buffer);
  tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));

  std::string frame_mode;
  nh.param("lidar_perception/cloud_frame_mode", frame_mode, std::string("auto"));
  T_body_to_cloud_ = Eigen::Matrix4f::Identity();
  needs_transform_ = false;

  if (frame_mode == "world") {
    // The cloud is already registered in the odometry/map frame. Ignore the
    // PointCloud2 frame_id because some drivers publish a misleading value.
    needs_transform_ = false;
    ROS_WARN("[LIOInterface] cloud_frame_mode=world: using %s as registered "
             "world/odom cloud and ignoring PointCloud2 frame_id",
             cloud_topic.c_str());
  } else if (frame_mode == "sensor") {
    // The cloud is raw sensor-frame data. Transform every synchronized scan by
    // T_map_body(odom) * T_body_cloud(configured mount extrinsic).
    double mount_pitch_deg = 0.0;
    double mount_yaw_deg = 0.0;
    std::vector<double> mount_offset;
    nh.param("lidar_perception/sensor_mount_pitch_deg", mount_pitch_deg, 0.0);
    nh.param("lidar_perception/sensor_mount_yaw_deg", mount_yaw_deg, 0.0);
    nh.param("lidar_perception/sensor_mount_offset", mount_offset,
             std::vector<double>());

    Eigen::AngleAxisf pitch_axis(M_PI / 180.0 * mount_pitch_deg,
                                 Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yaw_axis(M_PI / 180.0 * mount_yaw_deg,
                               Eigen::Vector3f::UnitZ());
    T_body_to_cloud_ = Eigen::Matrix4f::Identity();
    T_body_to_cloud_.block<3, 3>(0, 0) =
        (Eigen::Quaternionf(yaw_axis) * Eigen::Quaternionf(pitch_axis))
            .toRotationMatrix();
    if (mount_offset.size() == 3) {
      T_body_to_cloud_.block<3, 1>(0, 3) =
          Eigen::Vector3f(mount_offset[0], mount_offset[1], mount_offset[2]);
    }
    needs_transform_ = true;
    ROS_WARN("[LIOInterface] cloud_frame_mode=sensor: applying configured "
             "body->cloud mount transform pitch=%.2fdeg yaw=%.2fdeg offset=[%.3f %.3f %.3f]",
             mount_pitch_deg, mount_yaw_deg,
             mount_offset.size() == 3 ? mount_offset[0] : 0.0,
             mount_offset.size() == 3 ? mount_offset[1] : 0.0,
             mount_offset.size() == 3 ? mount_offset[2] : 0.0);
  } else if (frame_mode == "auto") {
    ROS_WARN("[LIOInterface] cloud_frame_mode=auto: inferring cloud transform "
             "from frame_id strings. Use explicit world/sensor mode for real flight.");

    ROS_INFO("[LIOInterface] Waiting for first odometry message on %s...",
             odom_topic.c_str());
    nav_msgs::Odometry::ConstPtr first_odom =
        ros::topic::waitForMessage<nav_msgs::Odometry>(odom_topic, nh,
                                                       ros::Duration(10.0));

    ROS_INFO("[LIOInterface] Waiting for first pointcloud message on %s...",
             cloud_topic.c_str());
    sensor_msgs::PointCloud2::ConstPtr first_cloud =
        ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
            cloud_topic, nh, ros::Duration(10.0));

    if (first_odom && first_cloud) {
      std::string map_frame = first_odom->header.frame_id;
      std::string body_frame = first_odom->child_frame_id;
      std::string cloud_frame = first_cloud->header.frame_id;

      ROS_INFO("[LIOInterface] Detected frames - Map: %s, Body: %s, Cloud: %s",
               map_frame.c_str(), body_frame.c_str(), cloud_frame.c_str());

      initializeTransform(map_frame, body_frame, cloud_frame);
    } else {
      ROS_WARN("[LIOInterface] Failed to receive initial messages, transform "
               "initialization skipped");
      needs_transform_ = false;
    }
  } else {
    ROS_FATAL("[LIOInterface] Invalid lidar_perception/cloud_frame_mode='%s'. "
              "Expected one of: auto, world, sensor.",
              frame_mode.c_str());
    ros::shutdown();
    exit(1);
  }

  // update_trigger_puber_ =
  //     nh.advertise<std_msgs::Empty>("/lio_interface/map_updated", 1);
  // cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(
  //     nh, cloud_topic, 5));
  // odom_sub_.reset(
  //     new message_filters::Subscriber<nav_msgs::Odometry>(nh, odom_topic, 1000));
  // sync_cloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyCloudOdom>(
  //     SyncPolicyCloudOdom(1), *cloud_sub_, *odom_sub_));
  // sync_cloud_odom_->registerCallback(
  //     boost::bind(&LIOInterface::updateCloudMapOdometry, this, _1, _2));
}

} // namespace fast_planner
