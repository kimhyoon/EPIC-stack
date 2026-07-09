/**
 * cloud_crop_bridge — ML-X 라이다 FOV 에뮬레이션용 crop 브릿지
 *
 * 목적: Mid-360(수평 360°) 실기에서 ML-X(전방 120° x 35°)를 흉내내기 위해,
 *   FAST-LIO 출력(cloud_topic, world frame)을 받아 "그 스캔 시점의 기체 자세"
 *   기준 FOV/거리로 잘라 cloud_crop/output_topic 으로 재발행한다.
 *   FAST-LIO 는 raw 센서 데이터를 그대로 쓰므로 localization 은 영향 없음 —
 *   EPIC(exploration_node)의 입력만 잘린다.
 *
 * 배선 (real.yaml 단일 소스, 폴백 금지):
 *   cloud_crop/enable: true  -> 이 노드가 crop 해서 재발행, exploration_node 도
 *                               같은 키를 읽어 crop 토픽을 구독 (수동 토픽 변경 불필요)
 *   cloud_crop/enable: false -> 이 노드는 즉시 자진 종료(clean exit),
 *                               exploration_node 는 원래 cloud_topic 을 구독
 *
 * 구현 노트:
 *  - cloud 는 world frame 이므로 FOV 판정은 /Odometry(=FAST-LIO body pose, 같은
 *    stamp 로 스캔마다 발행됨)와 ApproximateTime 동기화 후 body frame 으로 역변환해 수행.
 *    (Mid-360 lidar-IMU extrinsic 은 mm 수준 + R=I 라 body ~= lidar frame 으로 취급)
 *  - 출력 header(stamp/frame)는 입력 그대로 보존 — 하류 FSM 의 cloud+odom
 *    ApproximateTime 동기화가 깨지지 않게 하는 필수 조건.
 *  - PCL 변환 없이 PointCloud2 raw 바이트(point_step 블록)를 그대로 복사 —
 *    intensity 등 필드 레이아웃 불문 보존, 복사 1회.
 */
#include <Eigen/Dense>
#include <cmath>
#include <cstring>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace {

struct CropConfig {
  double half_h_rad;    // 수평 FOV 반각
  double fov_up_rad;    // 수직 상한 (elevation)
  double fov_down_rad;  // 수직 하한
  double min_range2;    // 제곱 비교용
  double max_range2;    // <=0 이면 무제한
  double yaw_offset_rad;
};

class CloudCropBridge {
public:
  CloudCropBridge(ros::NodeHandle &nh, const std::string &out_topic,
                  const std::string &cloud_topic, const std::string &odom_topic,
                  const CropConfig &cfg)
      : cfg_(cfg) {
    pub_ = nh.advertise<sensor_msgs::PointCloud2>(out_topic, 10);
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

  void cloudOdomCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud,
                         const nav_msgs::Odometry::ConstPtr &odom) {
    // 프레임 sanity check: cloud 는 odom 의 world frame 에 있어야 함
    if (!frame_checked_) {
      frame_checked_ = true;
      if (!cloud->header.frame_id.empty() &&
          cloud->header.frame_id != odom->header.frame_id)
        ROS_WARN("[CloudCrop] cloud frame '%s' != odom frame '%s' — crop is "
                 "wrong unless cloud is in the odom (world) frame.",
                 cloud->header.frame_id.c_str(),
                 odom->header.frame_id.c_str());
      if (cloud->is_bigendian)
        ROS_WARN("[CloudCrop] big-endian cloud unsupported (assuming LE)");
    }

    // x/y/z 필드 오프셋 (FLOAT32 가정 — FAST-LIO 출력)
    int ox = -1, oy = -1, oz = -1;
    for (const auto &f : cloud->fields) {
      if (f.datatype != sensor_msgs::PointField::FLOAT32) continue;
      if (f.name == "x") ox = f.offset;
      else if (f.name == "y") oy = f.offset;
      else if (f.name == "z") oz = f.offset;
    }
    if (ox < 0 || oy < 0 || oz < 0) {
      ROS_ERROR_THROTTLE(5.0, "[CloudCrop] no float32 x/y/z fields in cloud — skip");
      return;
    }

    // world -> body 역변환 준비
    const auto &p = odom->pose.pose.position;
    const auto &q = odom->pose.pose.orientation;
    const Eigen::Vector3f t(p.x, p.y, p.z);
    const Eigen::Matrix3f R_bw =
        Eigen::Quaternionf(q.w, q.x, q.y, q.z).toRotationMatrix().transpose();

    sensor_msgs::PointCloud2 out;
    out.header = cloud->header;  // stamp 보존 필수 (하류 sync)
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

    ROS_INFO_THROTTLE(5.0, "[CloudCrop] %zu -> %u pts (%.0f%%)", n_total,
                      out.width,
                      n_total ? 100.0 * out.width / n_total : 0.0);
  }

  CropConfig cfg_;
  ros::Publisher pub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  bool frame_checked_ = false;
};

} // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "cloud_crop_bridge");
  ros::NodeHandle nh("~");

  // enable 키 자체가 없으면 config 로드 실패로 간주 — 폴백 금지 정책
  bool enable = false;
  if (!nh.getParam("cloud_crop/enable", enable)) {
    ROS_FATAL("[CloudCrop] cloud_crop/enable not set in config yaml. "
              "REFUSING TO START - no fallback.");
    return 1;
  }
  if (!enable) {
    // 브릿지 꺼짐 = 정상 (EPIC 은 원래 cloud_topic 을 구독). 조용히 자진 종료.
    ROS_WARN("[CloudCrop] cloud_crop/enable=false — bridge disabled, exiting "
             "cleanly (EPIC subscribes raw cloud_topic).");
    return 0;
  }

  std::string cloud_topic, odom_topic, out_topic;
  if (!nh.getParam("cloud_topic", cloud_topic) || cloud_topic.empty() ||
      !nh.getParam("odometry_topic", odom_topic) || odom_topic.empty() ||
      !nh.getParam("cloud_crop/output_topic", out_topic) || out_topic.empty()) {
    ROS_FATAL("[CloudCrop] cloud_topic / odometry_topic / "
              "cloud_crop/output_topic not set in config yaml. "
              "REFUSING TO START - no fallback.");
    return 1;
  }
  if (out_topic == cloud_topic) {
    ROS_FATAL("[CloudCrop] output_topic == cloud_topic (%s) — refusing (loop).",
              out_topic.c_str());
    return 1;
  }

  // ML-XC 기본값: FOV 120° x 35°(대칭 가정), 감지거리 30 m
  double fov_h, fov_up, fov_down, min_range, max_range, yaw_offset;
  nh.param("cloud_crop/fov_horizontal", fov_h, 120.0);
  nh.param("cloud_crop/fov_up", fov_up, 17.5);
  nh.param("cloud_crop/fov_down", fov_down, -17.5);
  nh.param("cloud_crop/min_range", min_range, 0.0);
  nh.param("cloud_crop/max_range", max_range, 30.0);
  nh.param("cloud_crop/yaw_offset_deg", yaw_offset, 0.0);

  const double D2R = M_PI / 180.0;
  CropConfig cfg;
  cfg.half_h_rad = 0.5 * fov_h * D2R;
  cfg.fov_up_rad = fov_up * D2R;
  cfg.fov_down_rad = fov_down * D2R;
  cfg.min_range2 = min_range * min_range;
  cfg.max_range2 = max_range > 0.0 ? max_range * max_range : -1.0;
  cfg.yaw_offset_rad = yaw_offset * D2R;

  ROS_WARN("[CloudCrop] ON: %s -> %s | FOV h=%.1fdeg v=[%.1f, %.1f]deg "
           "range=[%.1f, %s] m yaw_offset=%.1fdeg (odom: %s)",
           cloud_topic.c_str(), out_topic.c_str(), fov_h, fov_down, fov_up,
           min_range,
           max_range > 0.0 ? std::to_string(max_range).c_str() : "inf",
           yaw_offset, odom_topic.c_str());

  CloudCropBridge bridge(nh, out_topic, cloud_topic, odom_topic, cfg);
  ros::spin();
  return 0;
}
