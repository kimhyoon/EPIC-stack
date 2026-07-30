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
 * ---------------------------------------------------------------------------
 * FRAME CONVENTION (read before touching any angle here)
 * ---------------------------------------------------------------------------
 *   ALL cloud_crop/fov_* and cloud_crop/yaw_offset_deg are expressed in the
 *   BODY frame (REP-103 ENU body: +x forward, +y left, +z up). There is NO
 *   hand-baked mounting correction in the yaml - do not "pre-tilt" fov_up /
 *   fov_down by the lidar mount angle. The physical mount tilt is handled by
 *   exactly one parameter, cloud_crop/odom_mount_pitch_deg, which is folded
 *   into the crop rotation matrix below.
 *
 *   Derivation of the crop rotation (see D2 in the design notes):
 *     - The odometry topic gives q = R_ws, i.e. sensor -> world.
 *       => p_s = R_ws^T * (p_w - t)
 *     - odom_mount_pitch_deg = theta is the lidar's forward-DOWN tilt. The
 *       sign convention is fixed by ws/epic/execution/odom_mount_comp_relay.py,
 *       which converts a sensor attitude into a body attitude with
 *           q_body = q_sensor (x) R_y(-theta)   =>   R_wb = R_ws * Ry(-theta)
 *       Therefore  R_sb = Ry(-theta)  and  R_bs = Ry(+theta).
 *       (Cross-check: that same relay rotates vectors with
 *        v_body.x = cos(th)*v.x + sin(th)*v.z, v_body.z = -sin(th)*v.x +
 *        cos(th)*v.z, which IS Ry(+theta) applied to a sensor-frame vector.)
 *       => p_b = Ry(+theta) * R_ws^T * (p_w - t)
 *     - The crop cone axis sits at azimuth yaw_offset in the body frame. Rather
 *       than subtracting yaw_offset from a per-point atan2, rotate the points:
 *           p_crop = Rz(-yaw_offset) * p_b
 *       so that the cone axis becomes +x in the crop frame.
 *   Final, computed ONCE per callback:
 *       R_crop = Rz(-yaw_offset) * Ry(+odom_mount_pitch) * R_ws^T
 *       p_crop = R_crop * (p_world - t)
 *   Ry does NOT preserve z or the xy-norm, so it MUST be inside the matrix; it
 *   cannot be applied as a post-correction on the elevation test. Rz DOES
 *   preserve both, so folding yaw_offset into the matrix leaves the elevation
 *   and range tests untouched.
 *
 * Implementation notes:
 *  - The input cloud is expected in the odometry/world frame.
 *  - Output header stamp/frame are preserved to keep downstream cloud/odom
 *    synchronization intact.
 *  - PointCloud2 point records are copied as raw point_step blocks by default,
 *    preserving field layout such as intensity (cloud_crop/output_fields).
 *
 * !! DO NOT COMPILE THIS FILE WITH -ffast-math !!
 *    The NaN/Inf rejection below relies on IEEE NaN comparison semantics
 *    (every comparison against NaN is false). -ffast-math lets GCC assume no
 *    NaNs exist and silently deletes that filter, letting garbage points reach
 *    EPIC's map. See CMakeLists.txt for the matching warning.
 */
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

namespace {

// Vertical bounds are turned into tan(); clamp away from the +-90 deg pole.
constexpr double kMaxElevDeg = 89.9;
// Window over which the periodic statistics line is aggregated.
constexpr double kStatWindowSec = 5.0;

struct CropConfig {
  // --- as configured, kept verbatim for the startup banner / logs ---
  double fov_h_deg;
  double fov_up_deg;
  double fov_down_deg;
  double yaw_offset_deg;
  double mount_pitch_deg;
  double min_range;
  double max_range;  // <= 0 means unlimited

  // --- derived, body frame, radians ---
  double half_h_rad;
  double fov_up_rad;
  double fov_down_rad;
  double yaw_offset_rad;
  double mount_pitch_rad;

  // --- hot-loop constants (float; no trig in the per-point path) ---
  float cos_half_h;  // horizontal test:  x >= cos_half_h * rho
  float tan_up;      // elevation test:   tan_down*rho <= z <= tan_up*rho
  float tan_down;
  float min_range2;
  // Squared max range. "Unlimited" is std::numeric_limits<float>::max() rather
  // than a negative sentinel on purpose: it lets the single range test also act
  // as the non-finite filter (see the crop loop).
  float max_range2;

  // --- visualization ---
  double visualization_range;
  double visualization_rate_hz;  // 0 = disabled
  int visualization_azimuth_samples;
  int visualization_elevation_samples;
};

struct Params {
  std::string cloud_topic;
  std::string odom_topic;
  std::string out_topic;
  std::string viz_topic;
  std::string sync_policy;  // "exact" | "approximate"
  int sync_queue;
  int cloud_queue;
  int odom_queue;
  int pub_queue;
  double sync_max_interval_sec;
  double watchdog_period_sec;  // 0 = disabled
  bool require_frame_match;
  bool profile_log;
  bool xyz_only;  // cloud_crop/output_fields == "xyz"
};

class CloudCropBridge {
public:
  CloudCropBridge(ros::NodeHandle &nh, const Params &p, const CropConfig &cfg)
      : cfg_(cfg), prm_(p) {
    // Rz(-yaw_offset) * Ry(+mount_pitch): the constant part of R_crop.
    R_align_ =
        (Eigen::AngleAxisf(static_cast<float>(-cfg_.yaw_offset_rad),
                           Eigen::Vector3f::UnitZ()) *
         Eigen::AngleAxisf(static_cast<float>(cfg_.mount_pitch_rad),
                           Eigen::Vector3f::UnitY()))
            .toRotationMatrix();
    // Ry(-mount_pitch): maps a body-frame vector to the sensor frame, used to
    // build the marker orientation (see buildMarkerTemplate/publishCropFov).
    R_y_neg_pitch_ = Eigen::AngleAxisf(static_cast<float>(-cfg_.mount_pitch_rad),
                                       Eigen::Vector3f::UnitY())
                         .toRotationMatrix();

    pub_ = nh.advertise<sensor_msgs::PointCloud2>(p.out_topic, p.pub_queue);
    // NOT latched: the marker is gated on subscriber count + a rate limit, and
    // a latched publisher would keep serializing (20 kB) with zero subscribers.
    fov_pub_ = nh.advertise<visualization_msgs::Marker>(p.viz_topic, 1);

    buildMarkerTemplate();
    if (p.xyz_only) buildXyzFields();

    const ros::TransportHints hints = ros::TransportHints().tcpNoDelay();
    cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(
        nh, p.cloud_topic, p.cloud_queue, hints));
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(
        nh, p.odom_topic, p.odom_queue, hints));

    // Liveness taps for the watchdog. These ride the existing subscriptions
    // (message_filters::Subscriber is a SimpleFilter), so no extra transport.
    cloud_sub_->registerCallback(
        boost::function<void(const sensor_msgs::PointCloud2::ConstPtr &)>(
            [this](const sensor_msgs::PointCloud2::ConstPtr &) {
              last_cloud_wall_ = ros::WallTime::now();
            }));
    odom_sub_->registerCallback(
        boost::function<void(const nav_msgs::Odometry::ConstPtr &)>(
            [this](const nav_msgs::Odometry::ConstPtr &) {
              last_odom_wall_ = ros::WallTime::now();
            }));

    if (p.sync_policy == "exact") {
      sync_exact_.reset(new message_filters::Synchronizer<ExactPolicy>(
          ExactPolicy(p.sync_queue), *cloud_sub_, *odom_sub_));
      sync_exact_->registerCallback(
          boost::bind(&CloudCropBridge::cloudOdomCallback, this, _1, _2));
    } else {
      sync_approx_.reset(new message_filters::Synchronizer<ApproxPolicy>(
          ApproxPolicy(p.sync_queue), *cloud_sub_, *odom_sub_));
      sync_approx_->setMaxIntervalDuration(
          ros::Duration(p.sync_max_interval_sec));
      sync_approx_->registerCallback(
          boost::bind(&CloudCropBridge::cloudOdomCallback, this, _1, _2));
    }

    start_wall_ = ros::WallTime::now();
    last_stat_wall_ = start_wall_;
    if (p.watchdog_period_sec > 0.0) {
      // WallTimer, not Timer: under bag replay the ROS clock comes from /clock,
      // and a stalled bag freezes ros::Time - exactly the case the watchdog
      // must report. Single-threaded ros::spin() means this callback never runs
      // concurrently with cloudOdomCallback, so no locking is needed.
      watchdog_ = nh.createWallTimer(ros::WallDuration(p.watchdog_period_sec),
                                     &CloudCropBridge::watchdogCallback, this);
    }
  }

private:
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2,
                                                    nav_msgs::Odometry>
      ExactPolicy;
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2, nav_msgs::Odometry>
      ApproxPolicy;

  // ---------------------------------------------------------------------
  // Visualization
  // ---------------------------------------------------------------------
  static geometry_msgs::Point toPoint(const Eigen::Vector3f &point) {
    geometry_msgs::Point out;
    out.x = point.x();
    out.y = point.y();
    out.z = point.z();
    return out;
  }

  static Eigen::Vector3f bodyFovPoint(double range, double azimuth,
                                      double elevation) {
    return Eigen::Vector3f(
        static_cast<float>(range * std::cos(elevation) * std::cos(azimuth)),
        static_cast<float>(range * std::cos(elevation) * std::sin(azimuth)),
        static_cast<float>(range * std::sin(elevation)));
  }

  void appendLine(const Eigen::Vector3f &from, const Eigen::Vector3f &to) {
    marker_.points.push_back(toPoint(from));
    marker_.points.push_back(toPoint(to));
  }

  // Built ONCE. The wireframe lives in the BODY frame with yaw_offset baked
  // into the azimuth span, so the per-scan work is only header + pose.
  void buildMarkerTemplate() {
    marker_.ns = "cloud_crop";
    marker_.id = 0;
    marker_.type = visualization_msgs::Marker::LINE_LIST;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.scale.x = 0.04;
    marker_.color.g = 0.95;
    marker_.color.b = 1.0;
    marker_.color.a = 1.0;
    marker_.pose.orientation.w = 1.0;

    const double far_range = cfg_.visualization_range;
    const double near_range = cfg_.min_range;
    const double azimuths[2] = {cfg_.yaw_offset_rad - cfg_.half_h_rad,
                                cfg_.yaw_offset_rad + cfg_.half_h_rad};
    const double elevations[2] = {cfg_.fov_down_rad, cfg_.fov_up_rad};
    const int as = cfg_.visualization_azimuth_samples;
    const int es = cfg_.visualization_elevation_samples;

    marker_.points.reserve(
        static_cast<size_t>(2) * ((es + 1) * as + (as + 1) * es + 4));

    // Draw the range boundary as a sampled spherical cap. Connecting only the
    // four corners would falsely imply that crop range has a planar end face.
    for (int ei = 0; ei <= es; ++ei) {
      const double elevation =
          elevations[0] + (elevations[1] - elevations[0]) * ei / es;
      Eigen::Vector3f previous = bodyFovPoint(far_range, azimuths[0], elevation);
      for (int ai = 1; ai <= as; ++ai) {
        const double azimuth =
            azimuths[0] + (azimuths[1] - azimuths[0]) * ai / as;
        const Eigen::Vector3f current =
            bodyFovPoint(far_range, azimuth, elevation);
        appendLine(previous, current);
        previous = current;
      }
    }
    for (int ai = 0; ai <= as; ++ai) {
      const double azimuth = azimuths[0] + (azimuths[1] - azimuths[0]) * ai / as;
      Eigen::Vector3f previous = bodyFovPoint(far_range, azimuth, elevations[0]);
      for (int ei = 1; ei <= es; ++ei) {
        const double elevation =
            elevations[0] + (elevations[1] - elevations[0]) * ei / es;
        const Eigen::Vector3f current =
            bodyFovPoint(far_range, azimuth, elevation);
        appendLine(previous, current);
        previous = current;
      }
    }
    for (const double elevation : elevations)
      for (const double azimuth : azimuths)
        appendLine(bodyFovPoint(near_range, azimuth, elevation),
                   bodyFovPoint(far_range, azimuth, elevation));
  }

  void buildXyzFields() {
    // 12 B / 3 float32 fields. Deliberately NOT padded to sizeof(pcl::PointXYZ)
    // (=16): pcl::fromROSMsg only takes its whole-cloud memcpy shortcut when the
    // coalesced field span equals BOTH point_step and sizeof(PointT)
    // (pcl/conversions.h:186-189). The x/y/z span is 12 B, so with a 16 B
    // point_step it still falls back to the per-point copy - the padding would
    // buy nothing and cost 33% more bytes on the wire.
    xyz_fields_.resize(3);
    const char *names[3] = {"x", "y", "z"};
    for (int i = 0; i < 3; ++i) {
      xyz_fields_[i].name = names[i];
      xyz_fields_[i].offset = 4u * i;
      xyz_fields_[i].datatype = sensor_msgs::PointField::FLOAT32;
      xyz_fields_[i].count = 1;
    }
  }

  void publishCropFov(const std_msgs::Header &header, const Eigen::Vector3f &t,
                      const Eigen::Matrix3f &R_ws) {
    // marker_.points are BODY-frame; marker_.header.frame_id stays the cloud's
    // (world) frame, so marker_.pose must carry body -> world.
    //   p_w = R_wb_eff * p_b + t, and from the crop derivation
    //   p_b = Ry(+theta) * R_ws^T * (p_w - t), hence
    //   R_wb_eff = (Ry(+theta) * R_ws^T)^T = R_ws * Ry(+theta)^T
    //            = R_ws * Ry(-theta)
    // which is exactly the q_body that odom_mount_comp_relay.py produces from
    // q_sensor - the two stay consistent by construction.
    const Eigen::Quaternionf q(R_ws * R_y_neg_pitch_);
    marker_.header = header;
    marker_.pose.position.x = t.x();
    marker_.pose.position.y = t.y();
    marker_.pose.position.z = t.z();
    marker_.pose.orientation.x = q.x();
    marker_.pose.orientation.y = q.y();
    marker_.pose.orientation.z = q.z();
    marker_.pose.orientation.w = q.w();
    fov_pub_.publish(marker_);
  }

  // ---------------------------------------------------------------------
  // Watchdog
  // ---------------------------------------------------------------------
  void watchdogCallback(const ros::WallTimerEvent &) {
    const ros::WallTime now = ros::WallTime::now();
    const double thr = prm_.watchdog_period_sec;
    const ros::WallTime since =
        last_pub_wall_.isZero() ? start_wall_ : last_pub_wall_;
    const double stalled = (now - since).toSec();
    if (stalled <= thr) return;

    // Deliberately NOT ROS_WARN_THROTTLE: that macro throttles on
    // ros::Time::now() (console.h:461-463), which freezes together with /clock
    // under sim time - i.e. it would go silent in precisely the stall this
    // watchdog exists to report. Throttle on the wall clock instead.
    const double warn_period = std::max(thr, 1.0);
    if (!last_warn_wall_.isZero() &&
        (now - last_warn_wall_).toSec() < warn_period)
      return;
    last_warn_wall_ = now;

    const bool cloud_alive =
        !last_cloud_wall_.isZero() && (now - last_cloud_wall_).toSec() <= thr;
    const bool odom_alive =
        !last_odom_wall_.isZero() && (now - last_odom_wall_).toSec() <= thr;

    const char *cause;
    if (!cloud_alive && !odom_alive)
      cause = "NO INPUT AT ALL (neither cloud nor odom arriving)";
    else if (!cloud_alive)
      cause = "CLOUD stream stalled (odom is alive)";
    else if (!odom_alive)
      cause = "ODOM stream stalled (cloud is alive)";
    else
      cause = "both inputs alive but the synchronizer emits NO pair "
              "(stamp mismatch -> check cloud_crop/sync_policy and "
              "cloud_crop/sync_max_interval_sec)";
    ROS_WARN("[CloudCrop] WATCHDOG: no output on '%s' for %.2f s - %s "
             "(EPIC's map is frozen while this lasts)",
             prm_.out_topic.c_str(), stalled, cause);
  }

  // ---------------------------------------------------------------------
  // Crop
  // ---------------------------------------------------------------------
  void cloudOdomCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud,
                         const nav_msgs::Odometry::ConstPtr &odom) {
    const bool prof = prm_.profile_log;
    const ros::WallTime t_begin = prof ? ros::WallTime::now() : ros::WallTime();

    // Frame sanity check: cloud should be in the odometry/world frame.
    if (!frame_checked_) {
      frame_checked_ = true;
      if (!cloud->header.frame_id.empty() &&
          cloud->header.frame_id != odom->header.frame_id) {
        if (prm_.require_frame_match) {
          ROS_FATAL("[CloudCrop] cloud frame '%s' != odom frame '%s' and "
                    "cloud_crop/require_frame_match=true - SHUTTING DOWN.",
                    cloud->header.frame_id.c_str(),
                    odom->header.frame_id.c_str());
          ros::shutdown();
          return;
        }
        ROS_WARN("[CloudCrop] cloud frame '%s' != odom frame '%s' - crop is "
                 "wrong unless cloud is in the odom (world) frame. "
                 "(set cloud_crop/require_frame_match=true to make this fatal)",
                 cloud->header.frame_id.c_str(),
                 odom->header.frame_id.c_str());
      }
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
      ROS_ERROR_THROTTLE(5.0,
                         "[CloudCrop] no float32 x/y/z fields in cloud - skip");
      return;
    }
    const size_t ps = cloud->point_step;
    if (ps == 0 || ps < static_cast<size_t>(std::max(ox, std::max(oy, oz))) + 4 ||
        cloud->data.size() <
            static_cast<size_t>(cloud->height) * cloud->row_step) {
      ROS_ERROR_THROTTLE(5.0,
                         "[CloudCrop] malformed cloud (point_step=%zu "
                         "row_step=%u height=%u data=%zu) - skip",
                         ps, cloud->row_step, cloud->height,
                         cloud->data.size());
      return;
    }
    // x/y/z contiguous -> one 12 B load instead of three 4 B loads.
    const bool xyz_contiguous = (oy == ox + 4 && oz == ox + 8);

    // R_crop = Rz(-yaw_offset) * Ry(+mount_pitch) * R_ws^T   (see file header)
    const auto &pos = odom->pose.pose.position;
    const auto &qq = odom->pose.pose.orientation;
    const Eigen::Vector3f t(pos.x, pos.y, pos.z);
    const Eigen::Matrix3f R_ws =
        Eigen::Quaternionf(qq.w, qq.x, qq.y, qq.z).normalized().toRotationMatrix();
    const Eigen::Matrix3f R_crop = R_align_ * R_ws.transpose();

    const size_t n_total =
        static_cast<size_t>(cloud->width) * static_cast<size_t>(cloud->height);
    const size_t ops = prm_.xyz_only ? 12u : ps;  // output point_step

    // Scratch grows to the high-water mark and is never shrunk, so the resize
    // below is a no-op after the first few scans. Writing straight into
    // out_.data instead would make libstdc++ value-initialize (memset) the
    // ~380 kB it re-exposes on every grow-back.
    const size_t need = n_total * ops;
    if (scratch_.size() < need) scratch_.resize(need);
    uint8_t *const base = scratch_.data();
    uint8_t *cur = base;

    // Hoist everything the loop reads through `this` / the message shared_ptr:
    // the memcpy stores into scratch_ may alias those members as far as the
    // compiler knows, so leaving them as member loads costs a reload per point.
    const float min_r2 = cfg_.min_range2;
    const float max_r2 = cfg_.max_range2;
    const float cos_h = cfg_.cos_half_h;
    const float tan_u = cfg_.tan_up;
    const float tan_d = cfg_.tan_down;
    const bool xyz_only = prm_.xyz_only;
    const uint32_t n_rows = cloud->height;
    const uint32_t n_cols = cloud->width;
    const size_t row_step = cloud->row_step;

    const uint8_t *src = cloud->data.data();
    for (uint32_t r = 0; r < n_rows; ++r) {
      const uint8_t *row = src + static_cast<size_t>(r) * row_step;
      for (uint32_t c = 0; c < n_cols; ++c) {
        const uint8_t *pt = row + static_cast<size_t>(c) * ps;
        float xyz[3];
        if (xyz_contiguous) {
          std::memcpy(xyz, pt + ox, 12);
        } else {
          std::memcpy(&xyz[0], pt + ox, 4);
          std::memcpy(&xyz[1], pt + oy, 4);
          std::memcpy(&xyz[2], pt + oz, 4);
        }

        const Eigen::Vector3f pb =
            R_crop * (Eigen::Vector3f(xyz[0], xyz[1], xyz[2]) - t);

        // Doubles as the non-finite filter: NaN propagates through the matvec
        // into r2, and every comparison against NaN is false, so the negated
        // range window rejects it. Inf is rejected by max_r2 == FLT_MAX. This
        // is why "unlimited" max_range is FLT_MAX and not a negative sentinel,
        // and why -ffast-math would break correctness (see file header).
        const float r2 = pb.squaredNorm();
        if (!(r2 >= min_r2 && r2 <= max_r2)) continue;

        const float rho = pb.head<2>().norm();
        // Body-origin points: atan2(0,0) used to return 0 and let them through
        // when min_range == 0. Degenerate and meaningless for a FOV test, and
        // the algebraic form below is undefined there, so drop them explicitly.
        if (rho <= 0.0f) continue;

        // Horizontal: |az| <= half_h  <=>  cos(az) >= cos(half_h)  <=>
        //             x >= cos(half_h) * rho     (rho > 0, half_h in [0, pi]).
        // Exact for cos_h < 0 (half_h > 90 deg) with no branch. Squaring to
        // drop the sqrt would need a sign branch, and rho is already computed.
        // NOTE: yaw_offset is now inside R_crop, so the old single-wrap
        // "az -= yaw_offset; if (az > pi) az -= 2pi" - which mis-classified
        // points whenever |az - yaw_offset| exceeded 2*pi - is gone entirely.
        if (pb.x() < cos_h * rho) continue;

        // Vertical: fov_down <= elev <= fov_up  <=>
        //           tan(fov_down)*rho <= z <= tan(fov_up)*rho   (rho > 0,
        //           |fov| clamped to 89.9 deg at startup so tan is finite).
        const float z = pb.z();
        if (z < tan_d * rho || z > tan_u * rho) continue;

        if (xyz_only) {
          std::memcpy(cur, xyz, 12);
        } else {
          std::memcpy(cur, pt, ps);
        }
        cur += ops;
      }
    }

    const size_t kept_bytes = static_cast<size_t>(cur - base);
    const uint32_t kept = static_cast<uint32_t>(kept_bytes / ops);

    const ros::WallTime t_loop = prof ? ros::WallTime::now() : ros::WallTime();

    out_.header = cloud->header;  // Preserve stamp for downstream sync.
    if (prm_.xyz_only) {
      out_.fields = xyz_fields_;
      out_.is_bigendian = false;  // we wrote host-order float32
    } else {
      out_.fields = cloud->fields;
      out_.is_bigendian = cloud->is_bigendian;
    }
    out_.point_step = static_cast<uint32_t>(ops);
    out_.height = 1;
    out_.width = kept;
    out_.row_step = static_cast<uint32_t>(kept_bytes);
    out_.is_dense = true;
    // assign() - NOT resize() - on purpose: assign copies over the existing
    // elements and never value-initializes, whereas growing resize() memsets
    // the newly exposed tail before we overwrite it.
    // (kept_bytes == 0 only happens for an empty/fully-cropped cloud, where
    // `base` may still be null; don't form base+0 on a null pointer.)
    if (kept_bytes)
      out_.data.assign(base, base + kept_bytes);
    else
      out_.data.clear();

    // *** BUFFER-REUSE SAFETY (out_ / scratch_ are members) ***
    // ros::Publisher::publish(const M&) (publisher.h:102-130) builds a
    // SerializedMessage WITHOUT setting m.type_info or m.message - unlike the
    // shared_ptr overload at publisher.h:67-97 which stores m.message = message
    // for intra-process hand-off. With a null type_info roscpp cannot defer, so
    // it serializes synchronously on THIS thread before publish() returns.
    // Reusing out_/scratch_ right after is therefore safe.
    // *** THIS BREAKS THE INSTANT ANYONE SWITCHES TO THE shared_ptr publish()
    // *** OVERLOAD OR NODELETIZES THIS NODE. Do neither without also going
    // *** back to allocating a fresh message per scan.
    pub_.publish(out_);
    last_pub_wall_ = ros::WallTime::now();

    const ros::WallTime t_pub = prof ? last_pub_wall_ : ros::WallTime();

    // Marker: gated on real subscribers AND a rate limit. Only the header and
    // pose change; the wireframe itself was built once in the constructor.
    if (cfg_.visualization_rate_hz > 0.0 && fov_pub_.getNumSubscribers() > 0) {
      const ros::WallTime now = ros::WallTime::now();
      if ((now - last_marker_wall_).toSec() >= 1.0 / cfg_.visualization_rate_hz) {
        last_marker_wall_ = now;
        publishCropFov(cloud->header, t, R_ws);
      }
    }

    const ros::WallTime t_mk = prof ? ros::WallTime::now() : ros::WallTime();

    // --- statistics (own wall-clock window so the timings stay meaningful
    //     under bag replay, where ros::Time follows /clock) ---
    ++stat_n_;
    stat_in_ += n_total;
    stat_out_ += kept;
    const double odom_dt_ms =
        (odom->header.stamp - cloud->header.stamp).toSec() * 1e3;
    const double age_ms =
        (ros::Time::now() - cloud->header.stamp).toSec() * 1e3;
    stat_odom_dt_sum_ += odom_dt_ms;
    stat_age_sum_ += age_ms;
    stat_age_max_ = std::max(stat_age_max_, age_ms);
    if (prof) {
      const double loop_ms = (t_loop - t_begin).toSec() * 1e3;
      const double pub_ms = (t_pub - t_loop).toSec() * 1e3;
      const double mk_ms = (t_mk - t_pub).toSec() * 1e3;
      stat_loop_sum_ += loop_ms;
      stat_loop_max_ = std::max(stat_loop_max_, loop_ms);
      stat_pub_sum_ += pub_ms;
      stat_pub_max_ = std::max(stat_pub_max_, pub_ms);
      stat_mk_sum_ += mk_ms;
      stat_mk_max_ = std::max(stat_mk_max_, mk_ms);
    }
    const ros::WallTime now_w = ros::WallTime::now();
    if ((now_w - last_stat_wall_).toSec() >= kStatWindowSec) {
      const double inv = 1.0 / static_cast<double>(stat_n_);
      const double in_avg = static_cast<double>(stat_in_) * inv;
      const double out_avg = static_cast<double>(stat_out_) * inv;
      if (prof)
        ROS_INFO("[CloudCrop] %.0f -> %.0f pts (%.0f%%) over %u scans | "
                 "odom_dt=%.2f ms age=%.1f/%.1f ms (avg/max) | "
                 "loop=%.3f/%.3f pub=%.3f/%.3f marker=%.3f/%.3f ms (avg/max)",
                 in_avg, out_avg, in_avg > 0.0 ? 100.0 * out_avg / in_avg : 0.0,
                 stat_n_, stat_odom_dt_sum_ * inv, stat_age_sum_ * inv,
                 stat_age_max_, stat_loop_sum_ * inv, stat_loop_max_,
                 stat_pub_sum_ * inv, stat_pub_max_, stat_mk_sum_ * inv,
                 stat_mk_max_);
      else
        ROS_INFO("[CloudCrop] %.0f -> %.0f pts (%.0f%%) over %u scans | "
                 "odom_dt=%.2f ms age=%.1f/%.1f ms (avg/max)",
                 in_avg, out_avg, in_avg > 0.0 ? 100.0 * out_avg / in_avg : 0.0,
                 stat_n_, stat_odom_dt_sum_ * inv, stat_age_sum_ * inv,
                 stat_age_max_);
      stat_n_ = 0;
      stat_in_ = stat_out_ = 0;
      stat_odom_dt_sum_ = stat_age_sum_ = stat_age_max_ = 0.0;
      stat_loop_sum_ = stat_loop_max_ = 0.0;
      stat_pub_sum_ = stat_pub_max_ = 0.0;
      stat_mk_sum_ = stat_mk_max_ = 0.0;
      last_stat_wall_ = now_w;
    }
  }

  CropConfig cfg_;
  Params prm_;
  Eigen::Matrix3f R_align_;       // Rz(-yaw_offset) * Ry(+mount_pitch)
  Eigen::Matrix3f R_y_neg_pitch_; // Ry(-mount_pitch), for the marker pose

  ros::Publisher pub_;
  ros::Publisher fov_pub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>>
      cloud_sub_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
  std::shared_ptr<message_filters::Synchronizer<ExactPolicy>> sync_exact_;
  std::shared_ptr<message_filters::Synchronizer<ApproxPolicy>> sync_approx_;
  ros::WallTimer watchdog_;

  // Reused output buffers - see the BUFFER-REUSE SAFETY note above.
  sensor_msgs::PointCloud2 out_;
  std::vector<uint8_t> scratch_;
  std::vector<sensor_msgs::PointField> xyz_fields_;
  visualization_msgs::Marker marker_;

  bool frame_checked_ = false;
  ros::WallTime start_wall_;
  ros::WallTime last_pub_wall_;
  ros::WallTime last_cloud_wall_;
  ros::WallTime last_odom_wall_;
  ros::WallTime last_warn_wall_;
  ros::WallTime last_marker_wall_;
  ros::WallTime last_stat_wall_;

  uint32_t stat_n_ = 0;
  size_t stat_in_ = 0;
  size_t stat_out_ = 0;
  double stat_odom_dt_sum_ = 0.0;
  double stat_age_sum_ = 0.0;
  double stat_age_max_ = 0.0;
  double stat_loop_sum_ = 0.0, stat_loop_max_ = 0.0;
  double stat_pub_sum_ = 0.0, stat_pub_max_ = 0.0;
  double stat_mk_sum_ = 0.0, stat_mk_max_ = 0.0;
};

// Cross-check the crop cone against EPIC's corridor cone. EPIC builds its FOV
// faces from  body_edge = lidar_perception/fov_* - lidar_perception/lidar_pitch
// (minco_planner/src/planner_manager.cpp:1022,1025), and its horizontal half
// angle from lidar_perception/yaw_fov. Since cloud_crop/fov_* are already body
// frame, they must equal those differences exactly.
// Returns false only when the mismatch is fatal.
bool checkFovAlignment(ros::NodeHandle &nh, const CropConfig &cfg,
                       bool strict) {
  double ep_up = 0.0, ep_down = 0.0, ep_yaw = 0.0, ep_pitch = 0.0;
  const bool have_up = nh.getParam("lidar_perception/fov_up", ep_up);
  const bool have_down = nh.getParam("lidar_perception/fov_down", ep_down);
  const bool have_yaw = nh.getParam("lidar_perception/yaw_fov", ep_yaw);
  const bool have_pitch = nh.getParam("lidar_perception/lidar_pitch", ep_pitch);
  if (!have_up || !have_down || !have_yaw) {
    ROS_INFO("[CloudCrop] lidar_perception/{fov_up,fov_down,yaw_fov} not "
             "visible from this node's namespace (up=%d down=%d yaw=%d) - "
             "skipping the crop/EPIC FOV consistency check.",
             (int)have_up, (int)have_down, (int)have_yaw);
    return true;
  }
  if (!have_pitch)
    ROS_WARN("[CloudCrop] lidar_perception/lidar_pitch missing - assuming 0.");

  const double want_up = ep_up - ep_pitch;
  const double want_down = ep_down - ep_pitch;
  const double tol = 0.01;
  const bool ok_up = std::fabs(want_up - cfg.fov_up_deg) <= tol;
  const bool ok_down = std::fabs(want_down - cfg.fov_down_deg) <= tol;
  const bool ok_yaw = std::fabs(ep_yaw - cfg.fov_h_deg) <= tol;

  ROS_WARN("[CloudCrop] FOV consistency: EPIC lidar_perception fov_up=%.3f "
           "fov_down=%.3f lidar_pitch=%.3f yaw_fov=%.3f -> body edges "
           "[%.3f, %.3f] x %.3f deg  ||  cloud_crop [%.3f, %.3f] x %.3f deg  "
           "=> up:%s down:%s yaw:%s",
           ep_up, ep_down, ep_pitch, ep_yaw, want_down, want_up, ep_yaw,
           cfg.fov_down_deg, cfg.fov_up_deg, cfg.fov_h_deg,
           ok_up ? "OK" : "MISMATCH", ok_down ? "OK" : "MISMATCH",
           ok_yaw ? "OK" : "MISMATCH");

  if (ok_up && ok_down && ok_yaw) return true;
  if (strict) {
    ROS_FATAL("[CloudCrop] crop cone does not match EPIC's corridor cone and "
              "cloud_crop/strict_fov_alignment=true. Required: "
              "cloud_crop/fov_up=%.3f fov_down=%.3f fov_horizontal=%.3f. "
              "REFUSING TO START.",
              want_up, want_down, ep_yaw);
    return false;
  }
  ROS_WARN("[CloudCrop] FOV mismatch tolerated "
           "(cloud_crop/strict_fov_alignment=false) - EPIC will plan through a "
           "corridor that does not match the cropped sensor FOV.");
  return true;
}

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

  Params prm;
  if (!nh.getParam("cloud_topic", prm.cloud_topic) || prm.cloud_topic.empty() ||
      !nh.getParam("odometry_topic", prm.odom_topic) ||
      prm.odom_topic.empty() ||
      !nh.getParam("cloud_crop/output_topic", prm.out_topic) ||
      prm.out_topic.empty()) {
    ROS_FATAL("[CloudCrop] cloud_topic / odometry_topic / "
              "cloud_crop/output_topic not set in config yaml. "
              "REFUSING TO START - no fallback.");
    return 1;
  }
  if (prm.out_topic == prm.cloud_topic) {
    ROS_FATAL("[CloudCrop] output_topic == cloud_topic (%s) - refusing (loop).",
              prm.out_topic.c_str());
    return 1;
  }

  nh.param<std::string>("cloud_crop/visualization_topic", prm.viz_topic,
                        "/debug/cloud_crop/fov");
  nh.param<std::string>("cloud_crop/sync_policy", prm.sync_policy,
                        std::string("approximate"));
  nh.param("cloud_crop/sync_queue", prm.sync_queue, 20);
  nh.param("cloud_crop/sync_max_interval_sec", prm.sync_max_interval_sec, 0.10);
  nh.param("cloud_crop/cloud_queue", prm.cloud_queue, 5);
  nh.param("cloud_crop/odom_queue", prm.odom_queue, 20);
  nh.param("cloud_crop/pub_queue", prm.pub_queue, 2);
  nh.param("cloud_crop/watchdog_period_sec", prm.watchdog_period_sec, 0.5);
  nh.param("cloud_crop/require_frame_match", prm.require_frame_match, false);
  nh.param("cloud_crop/profile_log", prm.profile_log, true);

  if (prm.sync_policy != "exact" && prm.sync_policy != "approximate") {
    ROS_FATAL("[CloudCrop] cloud_crop/sync_policy='%s' - must be 'exact' or "
              "'approximate'. REFUSING TO START - no fallback.",
              prm.sync_policy.c_str());
    return 1;
  }
  std::string output_fields;
  nh.param<std::string>("cloud_crop/output_fields", output_fields,
                        std::string("full"));
  if (output_fields != "full" && output_fields != "xyz") {
    ROS_FATAL("[CloudCrop] cloud_crop/output_fields='%s' - must be 'full' or "
              "'xyz'. REFUSING TO START - no fallback.",
              output_fields.c_str());
    return 1;
  }
  prm.xyz_only = (output_fields == "xyz");
  if (prm.sync_queue < 1 || prm.cloud_queue < 1 || prm.odom_queue < 1 ||
      prm.pub_queue < 1) {
    ROS_FATAL("[CloudCrop] queue sizes must be >= 1 (sync=%d cloud=%d odom=%d "
              "pub=%d). REFUSING TO START.",
              prm.sync_queue, prm.cloud_queue, prm.odom_queue, prm.pub_queue);
    return 1;
  }

  // ML-X defaults: 120 deg x 35 deg FOV, 30 m max range. All BODY frame.
  double fov_h, fov_up, fov_down, min_range, max_range, visualization_range,
      yaw_offset, mount_pitch, visualization_rate_hz;
  int visualization_azimuth_samples, visualization_elevation_samples;
  bool strict_fov_alignment;
  nh.param("cloud_crop/fov_horizontal", fov_h, 120.0);
  nh.param("cloud_crop/fov_up", fov_up, 17.5);
  nh.param("cloud_crop/fov_down", fov_down, -17.5);
  nh.param("cloud_crop/yaw_offset_deg", yaw_offset, 0.0);
  nh.param("cloud_crop/min_range", min_range, 0.0);
  nh.param("cloud_crop/max_range", max_range, 30.0);
  nh.param("cloud_crop/odom_mount_pitch_deg", mount_pitch, 0.0);
  nh.param("cloud_crop/visualization_range", visualization_range,
           max_range > 0.0 ? max_range : 10.0);
  nh.param("cloud_crop/visualization_rate_hz", visualization_rate_hz, 1.0);
  nh.param("cloud_crop/visualization_azimuth_samples",
           visualization_azimuth_samples, 24);
  nh.param("cloud_crop/visualization_elevation_samples",
           visualization_elevation_samples, 8);
  nh.param("cloud_crop/strict_fov_alignment", strict_fov_alignment, true);

  if (fov_h <= 0.0) {
    ROS_FATAL("[CloudCrop] cloud_crop/fov_horizontal=%.3f deg - must be > 0 "
              "(a non-positive horizontal FOV crops everything away). "
              "REFUSING TO START.",
              fov_h);
    return 1;
  }
  if (fov_down > fov_up)
    ROS_WARN("[CloudCrop] fov_down (%.3f) > fov_up (%.3f) - the vertical "
             "window is empty; every point will be dropped.",
             fov_down, fov_up);
  if (min_range < 0.0) {
    ROS_FATAL("[CloudCrop] cloud_crop/min_range=%.3f - must be >= 0.", min_range);
    return 1;
  }

  const double D2R = M_PI / 180.0;
  CropConfig cfg;
  cfg.fov_h_deg = fov_h;
  cfg.fov_up_deg = fov_up;
  cfg.fov_down_deg = fov_down;
  cfg.yaw_offset_deg = yaw_offset;
  cfg.mount_pitch_deg = mount_pitch;
  cfg.min_range = min_range;
  cfg.max_range = max_range;

  // Clamp 1: half_h must stay in [0, pi]. cos() turns around past pi, so a
  // fov_horizontal > 360 would start REJECTING points again.
  cfg.half_h_rad = std::min(0.5 * fov_h * D2R, M_PI);
  if (fov_h > 360.0)
    ROS_WARN("[CloudCrop] fov_horizontal=%.1f deg clamped to 360 deg.", fov_h);
  // Clamp 2: tan() blows up at +-90 deg.
  const double up_c = std::max(-kMaxElevDeg, std::min(kMaxElevDeg, fov_up));
  const double down_c = std::max(-kMaxElevDeg, std::min(kMaxElevDeg, fov_down));
  if (up_c != fov_up || down_c != fov_down)
    ROS_WARN("[CloudCrop] vertical FOV clamped to +-%.1f deg: [%.2f, %.2f] -> "
             "[%.2f, %.2f] (tan diverges at the pole).",
             kMaxElevDeg, fov_down, fov_up, down_c, up_c);
  cfg.fov_up_rad = up_c * D2R;
  cfg.fov_down_rad = down_c * D2R;
  cfg.yaw_offset_rad = yaw_offset * D2R;
  cfg.mount_pitch_rad = mount_pitch * D2R;

  cfg.cos_half_h = static_cast<float>(std::cos(cfg.half_h_rad));
  cfg.tan_up = static_cast<float>(std::tan(cfg.fov_up_rad));
  cfg.tan_down = static_cast<float>(std::tan(cfg.fov_down_rad));
  cfg.min_range2 = static_cast<float>(min_range * min_range);
  cfg.max_range2 = max_range > 0.0
                       ? static_cast<float>(max_range * max_range)
                       : std::numeric_limits<float>::max();

  cfg.visualization_range = std::max(visualization_range, min_range);
  cfg.visualization_rate_hz = std::max(0.0, visualization_rate_hz);
  cfg.visualization_azimuth_samples = std::max(2, visualization_azimuth_samples);
  cfg.visualization_elevation_samples =
      std::max(2, visualization_elevation_samples);

  // ---- startup banner: every effective parameter, so a yaml that failed to
  // ---- load is obvious at a glance.
  ROS_WARN("[CloudCrop] ================ cloud_crop_bridge ON ================");
  ROS_WARN("[CloudCrop]   cloud_topic          = %s", prm.cloud_topic.c_str());
  ROS_WARN("[CloudCrop]   odometry_topic       = %s", prm.odom_topic.c_str());
  ROS_WARN("[CloudCrop]   output_topic         = %s", prm.out_topic.c_str());
  ROS_WARN("[CloudCrop]   output_fields        = %s (point_step %s)",
           output_fields.c_str(), prm.xyz_only ? "12 (x,y,z)" : "= input");
  ROS_WARN("[CloudCrop]   --- crop cone, BODY frame (+x fwd, +z up) ---");
  ROS_WARN("[CloudCrop]   fov_horizontal       = %.3f deg (half=%.3f deg)",
           fov_h, cfg.half_h_rad / D2R);
  ROS_WARN("[CloudCrop]   fov_up / fov_down    = %.3f / %.3f deg", up_c, down_c);
  ROS_WARN("[CloudCrop]   yaw_offset_deg       = %.3f deg", yaw_offset);
  ROS_WARN("[CloudCrop]   min_range/max_range  = %.3f / %s m", min_range,
           max_range > 0.0 ? std::to_string(max_range).c_str() : "inf");
  ROS_WARN("[CloudCrop]   odom_mount_pitch_deg = %.3f deg  (%s)", mount_pitch,
           mount_pitch == 0.0
               ? "odom already reports BODY attitude"
               : "odom reports SENSOR attitude; R_bs=Ry(+pitch) applied");
  ROS_WARN("[CloudCrop]   --- sync ---");
  ROS_WARN("[CloudCrop]   sync_policy          = %s", prm.sync_policy.c_str());
  ROS_WARN("[CloudCrop]   sync_queue           = %d", prm.sync_queue);
  ROS_WARN("[CloudCrop]   sync_max_interval_s  = %.4f%s",
           prm.sync_max_interval_sec,
           prm.sync_policy == "exact" ? "  (unused: exact)" : "");
  ROS_WARN("[CloudCrop]   cloud/odom/pub queue = %d / %d / %d", prm.cloud_queue,
           prm.odom_queue, prm.pub_queue);
  ROS_WARN("[CloudCrop]   --- diagnostics ---");
  ROS_WARN("[CloudCrop]   watchdog_period_sec  = %.3f%s",
           prm.watchdog_period_sec,
           prm.watchdog_period_sec > 0.0 ? "" : "  (disabled)");
  ROS_WARN("[CloudCrop]   require_frame_match  = %s",
           prm.require_frame_match ? "true" : "false");
  ROS_WARN("[CloudCrop]   strict_fov_alignment = %s",
           strict_fov_alignment ? "true" : "false");
  ROS_WARN("[CloudCrop]   profile_log          = %s",
           prm.profile_log ? "true" : "false");
  ROS_WARN("[CloudCrop]   --- visualization ---");
  ROS_WARN("[CloudCrop]   visualization_topic  = %s", prm.viz_topic.c_str());
  ROS_WARN("[CloudCrop]   visualization_rate   = %.3f Hz%s",
           cfg.visualization_rate_hz,
           cfg.visualization_rate_hz > 0.0 ? "" : "  (disabled)");
  ROS_WARN("[CloudCrop]   visualization_range  = %.3f m",
           cfg.visualization_range);
  ROS_WARN("[CloudCrop]   viz az/el samples    = %d / %d",
           cfg.visualization_azimuth_samples,
           cfg.visualization_elevation_samples);
  ROS_WARN("[CloudCrop] =====================================================");

  if (!checkFovAlignment(nh, cfg, strict_fov_alignment)) return 1;

  CloudCropBridge bridge(nh, prm, cfg);
  ros::spin();
  return 0;
}
