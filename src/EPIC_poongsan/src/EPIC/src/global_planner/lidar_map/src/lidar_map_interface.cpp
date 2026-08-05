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
  transformed_cloud_puber_ =
      nh.advertise<sensor_msgs::PointCloud2>("/debug/mlx_cloud_world", 1);
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
  auto requirePositiveFinite = [&nh](const std::string &name, double &value) {
    if (!nh.getParam(name, value) || !std::isfinite(value) || value <= 0.0) {
      ROS_FATAL("[LIOInterface] required positive finite parameter '%s' is "
                "missing or invalid.",
                name.c_str());
      ros::shutdown();
      exit(1);
    }
  };
  requirePositiveFinite("numerical/vector_norm_eps", lp_->vector_norm_eps_);
  requirePositiveFinite("numerical/trig_gradient_eps",
                        lp_->trig_gradient_eps_);

  // A crop bridge keeps localization on the raw cloud while EPIC plans from
  // the configured ML-X-emulation output cloud.
  bool cloud_crop_enable = false;
  nh.param("cloud_crop/enable", cloud_crop_enable, false);
  if (cloud_crop_enable) {
    string cropped_topic;
    if (!nh.getParam("cloud_crop/output_topic", cropped_topic) ||
        cropped_topic.empty()) {
      ROS_FATAL("[LIOInterface] cloud_crop/enable=true but "
                "cloud_crop/output_topic is not configured.");
      ros::shutdown();
      exit(1);
    }
    ROS_WARN("[LIOInterface] cloud crop enabled: EPIC uses %s (raw: %s)",
             cropped_topic.c_str(), cloud_topic.c_str());
    cloud_topic = cropped_topic;
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
  // Keep older profiles valid while allowing raw sensor returns inside the
  // LiDAR's reliable minimum range to be rejected before map insertion.
  nh.param("lidar_perception/min_ray_length", lp_->min_ray_length_, 0.0);
  if (!std::isfinite(lp_->min_ray_length_) || lp_->min_ray_length_ < 0.0) {
    ROS_FATAL("[LIOInterface] lidar_perception/min_ray_length must be a "
              "non-negative finite distance.");
    ros::shutdown();
    exit(1);
  }
  requirePositiveFinite("lidar_perception/max_ray_length",
                        lp_->max_ray_length_);
  if (lp_->min_ray_length_ >= lp_->max_ray_length_) {
    ROS_FATAL("[LIOInterface] lidar_perception/min_ray_length (%.3f m) must "
              "be less than max_ray_length (%.3f m).",
              lp_->min_ray_length_, lp_->max_ray_length_);
    ros::shutdown();
    exit(1);
  }
  nh.param("lidar_perception/ray_clearing_enabled",
           lp_->ray_clearing_enabled_, false);
  nh.param("lidar_perception/ray_clearing_radius",
           lp_->ray_clearing_radius_, 0.10);
  nh.param("lidar_perception/ray_endpoint_margin",
           lp_->ray_endpoint_margin_, 0.20);
  if (lp_->ray_clearing_enabled_ &&
      (!std::isfinite(lp_->ray_clearing_radius_) ||
       lp_->ray_clearing_radius_ <= 0.0 ||
       !std::isfinite(lp_->ray_endpoint_margin_) ||
       lp_->ray_endpoint_margin_ < 0.0)) {
    ROS_FATAL("[LIOInterface] ray clearing requires a positive finite radius "
              "and a non-negative finite endpoint margin.");
    ros::shutdown();
    exit(1);
  }
  ROS_INFO("[LIOInterface] ray clearing: %s, radius=%.3f m, endpoint "
           "margin=%.3f m",
           lp_->ray_clearing_enabled_ ? "enabled" : "disabled",
           lp_->ray_clearing_radius_, lp_->ray_endpoint_margin_);
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
    // [feature: single-mount-source] 장착 회전의 단일 출처는 lidar_pitch/lidar_yaw 다.
    //
    // 예전에는 sensor_mount_pitch_deg / sensor_mount_yaw_deg 를 따로 받았지만,
    // 이 분기(cloud_frame_mode=sensor)의 변환이 T_map_body(odom) * T_body_cloud
    // 이므로 여기서는 odom 이 곧 body 다. 그리고 lidar_pitch 의 정의가 "odom
    // 프레임 대비 회전"이라, 이 모드에서는 lidar_pitch == mount pitch 가 항상
    // 성립한다 — 둘이 정당하게 달라질 수 있는 상황이 없다.
    // 같은 물리량을 두 곳에 적으면 한쪽만 고쳤을 때 점군 변환과 FOV 모델이 서로
    // 다른 자세를 가정하게 되므로, 회전은 lidar_pitch/lidar_yaw 하나만 쓴다.
    // (병진 offset 은 대응물이 없어 그대로 별도 파라미터로 유지)
    const double mount_pitch_deg = lp_->lidar_pitch_;
    const double mount_yaw_deg = lp_->lidar_yaw_;
    std::vector<double> mount_offset;
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
    ROS_WARN("[LIOInterface] cloud_frame_mode=sensor: applying body->cloud mount "
             "transform from lidar_pitch/lidar_yaw pitch=%.2fdeg yaw=%.2fdeg "
             "offset=[%.3f %.3f %.3f]",
             mount_pitch_deg, mount_yaw_deg,
             mount_offset.size() == 3 ? mount_offset[0] : 0.0,
             mount_offset.size() == 3 ? mount_offset[1] : 0.0,
             mount_offset.size() == 3 ? mount_offset[2] : 0.0);
    ROS_INFO("[LIOInterface] sensor minimum-range filter: rejecting returns "
             "at or below %.3f m",
             lp_->min_ray_length_);
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
