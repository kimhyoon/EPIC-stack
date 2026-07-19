/**
 * cloud_crop_bridge - ML-X LiDAR FOV emulation crop bridge
 *
 * Purpose:
 *   Crop a raw wide-FOV Mid-360 style cloud into an ML-X-style forward FOV
 *   before EPIC consumes it. FAST-LIO still uses the raw sensor input, so
 *   localization is not affected; only the EPIC exploration input is cropped.
 *
 * Wiring:
 *   cloud_crop/enable: true  -> crop and republish to cloud_crop/output_topic.
 *   cloud_crop/enable: false -> exit cleanly; EPIC subscribes to cloud_topic.
 *
 * Implementation notes:
 *  - The input cloud is expected in the odometry/world frame. Points are
 *    transformed into the body frame using the synchronized odometry pose for
 *    FOV testing.
 *  - Output header stamp/frame are preserved to keep downstream cloud/odom
 *    synchronization intact.
 *  - PointCloud2 point records are copied as raw point_step blocks, preserving
 *    field layout such as intensity.
 */
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

namespace {

struct CropConfig {
  double half_h_rad;    // Horizontal FOV half angle.
  double fov_up_rad;    // Upper vertical elevation bound.
  double fov_down_rad;  // Lower vertical elevation bound.
  double min_range2;    // Squared minimum range.
  double max_range2;    // Squared maximum range; <= 0 means unlimited.
  double visualization_range;
  double yaw_offset_rad;
};

class CloudCropBridge {
public:
  CloudCropBridge(ros::NodeHandle &nh, const std::string &out_topic,
                  const std::string &cloud_topic, const std::string &odom_topic,
                  const std::string &fov_marker_topic, const CropConfig &cfg)
      : cfg_(cfg) {
    pub_ = nh.advertise<sensor_msgs::PointCloud2>(out_topic, 10);
    fov_pub_ = nh.advertise<visualization_msgs::Marker>(fov_marker_topic, 1, true);
    cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(
        nh, cloud_topic, 5));
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(
        nh, odom_topic, 100));
    sync_.reset(new message_filters::Synchronizer<SyncPolicy>(
        SyncPolicy(20), *cloud_sub_, *odom_sub_));
    sync_->registerCallback(
        boost::bind(&CloudCropBridge::cloudOdomCallback, this, _1, _2));
  }

private:
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2, nav_msgs::Odometry>
      SyncPolicy;

  static geometry_msgs::Point toPoint(const Eigen::Vector3f &point) {
    geometry_msgs::Point out;
    out.x = point.x();
    out.y = point.y();
    out.z = point.z();
    return out;
  }

  static void appendLine(visualization_msgs::Marker &marker,
                         const Eigen::Vector3f &from,
                         const Eigen::Vector3f &to) {
    marker.points.push_back(toPoint(from));
    marker.points.push_back(toPoint(to));
  }

  void publishCropFov(const std_msgs::Header &header,
                       const Eigen::Vector3f &origin,
                       const Eigen::Matrix3f &R_wb) {
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.ns = "cloud_crop";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.04;
    marker.color.g = 0.95;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    const double far_range = cfg_.visualization_range;
    const double near_range = std::sqrt(cfg_.min_range2);
    const double azimuths[2] = {
        cfg_.yaw_offset_rad - cfg_.half_h_rad,
        cfg_.yaw_offset_rad + cfg_.half_h_rad};
    const double elevations[2] = {cfg_.fov_down_rad, cfg_.fov_up_rad};

    std::array<Eigen::Vector3f, 4> near_corners;
    std::array<Eigen::Vector3f, 4> far_corners;
    int index = 0;
    for (const double elevation : elevations) {
      for (const double azimuth : azimuths) {
        const Eigen::Vector3f unit_body(
            std::cos(elevation) * std::cos(azimuth),
            std::cos(elevation) * std::sin(azimuth), std::sin(elevation));
        near_corners[index] = origin + R_wb * (near_range * unit_body);
        far_corners[index] = origin + R_wb * (far_range * unit_body);
        ++index;
      }
    }

    for (size_t i = 0; i < far_corners.size(); ++i) {
      appendLine(marker, near_corners[i], far_corners[i]);
    }
    const int face_edges[4][2] = {{0, 1}, {1, 3}, {3, 2}, {2, 0}};
    for (const auto &edge : face_edges) {
      appendLine(marker, far_corners[edge[0]], far_corners[edge[1]]);
      if (near_range > 0.0) {
        appendLine(marker, near_corners[edge[0]], near_corners[edge[1]]);
      }
    }

    fov_pub_.publish(marker);
  }

  void cloudOdomCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud,
                         const nav_msgs::Odometry::ConstPtr &odom) {
    // Frame sanity check: cloud should be in the odometry/world frame.
    if (!frame_checked_) {
      frame_checked_ = true;
      if (!cloud->header.frame_id.empty() &&
          cloud->header.frame_id != odom->header.frame_id)
        ROS_WARN("[CloudCrop] cloud frame '%s' != odom frame '%s' - crop is "
                 "wrong unless cloud is in the odom (world) frame.",
                 cloud->header.frame_id.c_str(),
                 odom->header.frame_id.c_str());
      if (cloud->is_bigendian)
        ROS_WARN("[CloudCrop] big-endian cloud unsupported (assuming LE)");
    }

    // x/y/z field offsets. FAST-LIO output is expected as FLOAT32.
    int ox = -1, oy = -1, oz = -1;
    for (const auto &f : cloud->fields) {
      if (f.datatype != sensor_msgs::PointField::FLOAT32) continue;
      if (f.name == "x") ox = f.offset;
      else if (f.name == "y") oy = f.offset;
      else if (f.name == "z") oz = f.offset;
    }
    if (ox < 0 || oy < 0 || oz < 0) {
      ROS_ERROR_THROTTLE(5.0, "[CloudCrop] no float32 x/y/z fields in cloud - skip");
      return;
    }

    // Prepare world-to-body transform.
    const auto &p = odom->pose.pose.position;
    const auto &q = odom->pose.pose.orientation;
    const Eigen::Vector3f t(p.x, p.y, p.z);
    const Eigen::Matrix3f R_bw =
        Eigen::Quaternionf(q.w, q.x, q.y, q.z).toRotationMatrix().transpose();
    const Eigen::Matrix3f R_wb = R_bw.transpose();

    sensor_msgs::PointCloud2 out;
    out.header = cloud->header;  // Preserve stamp for downstream sync.
    out.fields = cloud->fields;
    out.is_bigendian = cloud->is_bigendian;
    out.point_step = cloud->point_step;
    out.height = 1;
    out.is_dense = true;
    out.data.reserve(cloud->data.size());

    const uint8_t *src = cloud->data.data();
    const size_t n_total = (size_t)cloud->width * cloud->height;
    for (uint32_t r = 0; r < cloud->height; ++r) {
      const uint8_t *row = src + (size_t)r * cloud->row_step;
      for (uint32_t c = 0; c < cloud->width; ++c) {
        const uint8_t *pt = row + (size_t)c * cloud->point_step;
        float xyz[3];
        std::memcpy(&xyz[0], pt + ox, 4);
        std::memcpy(&xyz[1], pt + oy, 4);
        std::memcpy(&xyz[2], pt + oz, 4);
        if (!std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) ||
            !std::isfinite(xyz[2]))
          continue;

        const Eigen::Vector3f pb =
            R_bw * (Eigen::Vector3f(xyz[0], xyz[1], xyz[2]) - t);
        const double r2 = pb.squaredNorm();
        if (r2 < cfg_.min_range2) continue;
        if (cfg_.max_range2 > 0.0 && r2 > cfg_.max_range2) continue;

        double az = std::atan2(pb.y(), pb.x()) - cfg_.yaw_offset_rad;
        if (az > M_PI) az -= 2.0 * M_PI;
        else if (az < -M_PI) az += 2.0 * M_PI;
        if (std::fabs(az) > cfg_.half_h_rad) continue;

        const double elev = std::atan2(pb.z(), pb.head<2>().norm());
        if (elev < cfg_.fov_down_rad || elev > cfg_.fov_up_rad) continue;

        out.data.insert(out.data.end(), pt, pt + cloud->point_step);
      }
    }
    out.width = cloud->point_step ? out.data.size() / cloud->point_step : 0;
    out.row_step = out.data.size();
    pub_.publish(out);
    publishCropFov(cloud->header, t, R_wb);

    ROS_INFO_THROTTLE(5.0, "[CloudCrop] %zu -> %u pts (%.0f%%)", n_total,
                      out.width,
                      n_total ? 100.0 * out.width / n_total : 0.0);
  }

  CropConfig cfg_;
  ros::Publisher pub_;
  ros::Publisher fov_pub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  bool frame_checked_ = false;
};

} // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "cloud_crop_bridge");
  ros::NodeHandle nh("~");

  // Missing enable means the config was not loaded; do not silently fall back.
  bool enable = false;
  if (!nh.getParam("cloud_crop/enable", enable)) {
    ROS_FATAL("[CloudCrop] cloud_crop/enable not set in config yaml. "
              "REFUSING TO START - no fallback.");
    return 1;
  }
  if (!enable) {
    // Disabled bridge is a normal mode; EPIC subscribes to raw cloud_topic.
    ROS_WARN("[CloudCrop] cloud_crop/enable=false - bridge disabled, exiting "
             "cleanly (EPIC subscribes raw cloud_topic).");
    return 0;
  }

  std::string cloud_topic, odom_topic, out_topic, fov_marker_topic;
  if (!nh.getParam("cloud_topic", cloud_topic) || cloud_topic.empty() ||
      !nh.getParam("odometry_topic", odom_topic) || odom_topic.empty() ||
      !nh.getParam("cloud_crop/output_topic", out_topic) || out_topic.empty()) {
    ROS_FATAL("[CloudCrop] cloud_topic / odometry_topic / "
              "cloud_crop/output_topic not set in config yaml. "
              "REFUSING TO START - no fallback.");
    return 1;
  }
  if (out_topic == cloud_topic) {
    ROS_FATAL("[CloudCrop] output_topic == cloud_topic (%s) - refusing (loop).",
              out_topic.c_str());
    return 1;
  }
  nh.param<std::string>("cloud_crop/visualization_topic", fov_marker_topic,
                        "/debug/cloud_crop/fov");

  // ML-XC defaults: 120 deg x 35 deg FOV, 30 m max range.
  double fov_h, fov_up, fov_down, min_range, max_range, visualization_range,
      yaw_offset;
  nh.param("cloud_crop/fov_horizontal", fov_h, 120.0);
  nh.param("cloud_crop/fov_up", fov_up, 17.5);
  nh.param("cloud_crop/fov_down", fov_down, -17.5);
  nh.param("cloud_crop/min_range", min_range, 0.0);
  nh.param("cloud_crop/max_range", max_range, 30.0);
  nh.param("cloud_crop/visualization_range", visualization_range,
           max_range > 0.0 ? max_range : 10.0);
  nh.param("cloud_crop/yaw_offset_deg", yaw_offset, 0.0);

  const double D2R = M_PI / 180.0;
  CropConfig cfg;
  cfg.half_h_rad = 0.5 * fov_h * D2R;
  cfg.fov_up_rad = fov_up * D2R;
  cfg.fov_down_rad = fov_down * D2R;
  cfg.min_range2 = min_range * min_range;
  cfg.max_range2 = max_range > 0.0 ? max_range * max_range : -1.0;
  cfg.visualization_range = std::max(visualization_range, min_range);
  cfg.yaw_offset_rad = yaw_offset * D2R;

  ROS_WARN("[CloudCrop] ON: %s -> %s | FOV h=%.1fdeg v=[%.1f, %.1f]deg "
           "range=[%.1f, %s] m yaw_offset=%.1fdeg (odom: %s, fov: %s)",
           cloud_topic.c_str(), out_topic.c_str(), fov_h, fov_down, fov_up,
           min_range,
           max_range > 0.0 ? std::to_string(max_range).c_str() : "inf",
           yaw_offset, odom_topic.c_str(), fov_marker_topic.c_str());

  CloudCropBridge bridge(nh, out_topic, cloud_topic, odom_topic,
                          fov_marker_topic, cfg);
  ros::spin();
  return 0;
}
