// px4_ctrl_bridge.cpp
//
// Bridges EPIC's /position_cmd (quadrotor_msgs::PositionCommand, a full-state
// reference: pos + vel + acc + yaw + yaw_dot in the ENU "world" frame) to a PX4
// flight controller via MAVROS OFFBOARD control.
//
// It republishes each command as mavros_msgs::PositionTarget on
// /mavros/setpoint_raw/local and lets PX4's internal position/attitude/rate
// controllers do the actual tracking. cascadePID + the MARSIM dynamics loop are
// NOT used on the real vehicle.
//
// Safety flow:
//   1. Before any /position_cmd arrives, stream a "hold" setpoint at the current
//      odometry pose (>2 Hz) so PX4 will accept an OFFBOARD switch.
//   2. Optionally auto-request OFFBOARD + arm (auto_arm:=true), or do it manually
//      from a radio / QGroundControl (auto_arm:=false, the default).
//   3. Once /position_cmd flows, forward it. If the command stream goes stale
//      (planner died / heartbeat lost), fall back to holding the last position.

#include <ros/ros.h>
#include <Eigen/Eigen>

#include <quadrotor_msgs/PositionCommand.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>

// ---- state -----------------------------------------------------------------
static mavros_msgs::State          px4_state_;
static nav_msgs::Odometry          odom_;
static bool                        have_odom_  = false;
static bool                        have_cmd_   = false;
static ros::Time                   last_cmd_stamp_;
static quadrotor_msgs::PositionCommand last_cmd_;

// ---- reactive-avoidance MUX state ------------------------------------------
static int                         avoid_flag_     = 0;     // last /FSM_flag_avoidance value (1 = obstacle close)
static bool                        have_flag_      = false;
static ros::Time                   last_flag_stamp_;
static mavros_msgs::PositionTarget last_avoid_;             // last /target_avoidance setpoint
static bool                        have_avoid_     = false;
static ros::Time                   last_avoid_stamp_;
static bool                        avoid_active_prev_ = false;  // for 1->0 edge detection

// ---- params ----------------------------------------------------------------
static bool   auto_arm_        = false;  // auto-request OFFBOARD + arm
static double cmd_timeout_     = 0.5;    // [s] treat cmd stream as stale after this
static bool   use_accel_ff_    = false;  // forward acceleration feed-forward to PX4
static bool   use_yawrate_     = true;   // forward yaw_dot, else hold yaw only

// ---- avoidance MUX params --------------------------------------------------
static bool   enable_avoidance_  = true;  // master switch for the reactive-avoidance MUX
static double avoid_cmd_timeout_ = 0.3;   // [s] /target_avoidance stale-after (node emits ~40 Hz)
static double flag_timeout_      = 0.5;   // [s] /FSM_flag_avoidance stale-after (silent -> revert to EPIC)
static bool   replan_on_release_ = true;  // publish /planning/replan on the avoidance 1->0 edge

// ---- pub / clients ---------------------------------------------------------
static ros::Publisher       sp_pub_;
static ros::ServiceClient   arming_client_;
static ros::ServiceClient   set_mode_client_;
static ros::Publisher       replan_pub_;   // /planning/replan (std_msgs::Empty) — EPIC resync on release

void stateCb(const mavros_msgs::State::ConstPtr& msg) { px4_state_ = *msg; }

void odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
  odom_ = *msg;
  have_odom_ = true;
}

void cmdCb(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
  last_cmd_       = *msg;
  last_cmd_stamp_ = ros::Time::now();
  have_cmd_       = true;
}

// Reactive-avoidance inputs (published by the local_avoidance node).
void flagCb(const std_msgs::Int16::ConstPtr& msg) {
  avoid_flag_      = msg->data;
  last_flag_stamp_ = ros::Time::now();
  have_flag_       = true;
}

void avoidCb(const mavros_msgs::PositionTarget::ConstPtr& msg) {
  last_avoid_       = *msg;
  last_avoid_stamp_ = ros::Time::now();
  have_avoid_       = true;
}

// Build a PositionTarget. All values are in ENU; MAVROS converts to NED for PX4.
mavros_msgs::PositionTarget makeSetpoint(const Eigen::Vector3d& p,
                                         const Eigen::Vector3d& v,
                                         const Eigen::Vector3d& a,
                                         double yaw, double yaw_rate,
                                         bool full_state) {
  mavros_msgs::PositionTarget sp;
  sp.header.stamp    = ros::Time::now();
  sp.header.frame_id = "map";
  // FRAME_LOCAL_NED is the MAVROS local-frame constant; MAVROS still expects
  // ENU on this topic and flips it to NED internally.
  sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

  // type_mask: a SET bit means IGNORE that field.
  uint16_t mask = 0;
  using PT = mavros_msgs::PositionTarget;
  if (full_state) {
    mask |= PT::IGNORE_VX | PT::IGNORE_VY | PT::IGNORE_VZ;   
    // ← 추가: 속도 무시 (위치만 추종) 

    if (!use_accel_ff_) mask |= PT::IGNORE_AFX | PT::IGNORE_AFY | PT::IGNORE_AFZ;
    if (use_yawrate_)   mask |= PT::IGNORE_YAW;       // command yaw_rate
    else                mask |= PT::IGNORE_YAW_RATE;  // command yaw
  } else {
    // hold mode: position + yaw only
    mask |= PT::IGNORE_VX | PT::IGNORE_VY | PT::IGNORE_VZ |
            PT::IGNORE_AFX | PT::IGNORE_AFY | PT::IGNORE_AFZ |
            PT::IGNORE_YAW_RATE;
  }
  sp.type_mask = mask;

  sp.position.x = p.x();  sp.position.y = p.y();  sp.position.z = p.z();
  sp.velocity.x = v.x();  sp.velocity.y = v.y();  sp.velocity.z = v.z();
  sp.acceleration_or_force.x = a.x();
  sp.acceleration_or_force.y = a.y();
  sp.acceleration_or_force.z = a.z();
  sp.yaw      = yaw;
  sp.yaw_rate = yaw_rate;
  return sp;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "px4_ctrl_bridge");
  ros::NodeHandle nh("~");

  // odom 토픽은 real.yaml 의 odometry_topic (exploration_node ns 로 로드) 이
  // 유일한 소스다. 폴백 절대 금지 — 실비행에서 다른 토픽(다른 좌표계)을
  // 조용히 구독하면 추락하므로, 파라미터가 없으면 시작 자체를 거부한다.
  std::string odom_topic;
  if (!ros::param::get("/exploration_node/odometry_topic", odom_topic) ||
      odom_topic.empty()) {
    ROS_FATAL("[px4_bridge] /exploration_node/odometry_topic not set "
              "(real.yaml not loaded?). REFUSING TO START - no fallback.");
    return 1;
  }
  ROS_INFO("[px4_bridge] odom topic: %s", odom_topic.c_str());
  nh.param("auto_arm",     auto_arm_,    auto_arm_);
  nh.param("cmd_timeout",  cmd_timeout_, cmd_timeout_);
  nh.param("use_accel_ff", use_accel_ff_, use_accel_ff_);
  nh.param("use_yawrate",  use_yawrate_,  use_yawrate_);
  nh.param("enable_avoidance",  enable_avoidance_,  enable_avoidance_);
  nh.param("avoid_cmd_timeout", avoid_cmd_timeout_, avoid_cmd_timeout_);
  nh.param("flag_timeout",      flag_timeout_,      flag_timeout_);
  nh.param("replan_on_release", replan_on_release_, replan_on_release_);
  // real.yaml 마스터 스위치(local_avoidance/enable, exploration_node ns 로드)와 AND.
  // false 면 회피 MUX 를 하드 차단 -> /target_avoidance 가 와도 EPIC 명령만 통과.
  {
    bool yaml_avoid = true;
    if (ros::param::get("/exploration_node/local_avoidance/enable", yaml_avoid) &&
        !yaml_avoid) {
      enable_avoidance_ = false;
      ROS_WARN("[px4_ctrl_bridge] avoidance MUX disabled by real.yaml "
               "(local_avoidance/enable=false)");
    }
  }

  ros::NodeHandle gnh;  // global handle for shared topics
  ros::Subscriber state_sub = gnh.subscribe("/mavros/state", 10, stateCb);
  ros::Subscriber odom_sub  = gnh.subscribe(odom_topic, 50, odomCb);
  ros::Subscriber cmd_sub   = gnh.subscribe("/position_cmd", 50, cmdCb);
  ros::Subscriber flag_sub  = gnh.subscribe("/FSM_flag_avoidance", 10, flagCb);
  ros::Subscriber avoid_sub = gnh.subscribe("/target_avoidance", 10, avoidCb);

  sp_pub_ = gnh.advertise<mavros_msgs::PositionTarget>(
      "/mavros/setpoint_raw/local", 50);
  replan_pub_ = gnh.advertise<std_msgs::Empty>("/planning/replan", 10);
  arming_client_   = gnh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  set_mode_client_ = gnh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

  // PX4 OFFBOARD requires setpoints already streaming; run the loop at 100 Hz.
  ros::Rate rate(100.0);

  ROS_INFO("[px4_bridge] waiting for FCU connection & odometry ...");
  while (ros::ok() && (!px4_state_.connected || !have_odom_)) {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("[px4_bridge] FCU connected, odometry up. auto_arm=%d", (int)auto_arm_);

  mavros_msgs::SetMode offb_req;  offb_req.request.custom_mode = "OFFBOARD";
  mavros_msgs::CommandBool arm_req; arm_req.request.value = true;
  ros::Time last_try = ros::Time::now();

  while (ros::ok()) {
    ros::spinOnce();

    // Optional automatic mode switch + arm. In production prefer doing this from
    // the safety pilot's transmitter so a human stays in the loop.
    if (auto_arm_ && (ros::Time::now() - last_try).toSec() > 1.0) {
      if (px4_state_.mode != "OFFBOARD") {
        if (set_mode_client_.call(offb_req) && offb_req.response.mode_sent)
          ROS_INFO("[px4_bridge] OFFBOARD requested");
      } else if (!px4_state_.armed) {
        if (arming_client_.call(arm_req) && arm_req.response.success)
          ROS_INFO("[px4_bridge] vehicle armed");
      }
      last_try = ros::Time::now();
    }

    ros::Time now = ros::Time::now();

    // ---- reactive-avoidance MUX --------------------------------------------
    // Freshness of each command source.
    const bool cmd_fresh =
        have_cmd_   && (now - last_cmd_stamp_).toSec()   < cmd_timeout_;
    const bool flag_fresh =
        have_flag_  && (now - last_flag_stamp_).toSec()  < flag_timeout_;
    const bool avoid_fresh =
        have_avoid_ && (now - last_avoid_stamp_).toSec() < avoid_cmd_timeout_;

    // Avoidance takes over only if enabled AND the flag topic is alive AND it
    // says "obstacle close" (==1) AND a fresh escape target exists. If the flag
    // topic itself goes silent, fall back to EPIC (stale-safe).
    const bool avoid_active =
        enable_avoidance_ && flag_fresh && (avoid_flag_ == 1) && avoid_fresh;

    // Falling edge (avoidance released): nudge EPIC to truncate its now-stale
    // trajectory. The FSM-aware Phase 2 hook also replans from the current
    // pose; this is belt-and-suspenders so the drone never snaps far back.
    if (avoid_active_prev_ && !avoid_active && replan_on_release_) {
      replan_pub_.publish(std_msgs::Empty());
      ROS_WARN("[px4_bridge] avoidance released -> published /planning/replan");
    }
    avoid_active_prev_ = avoid_active;

    mavros_msgs::PositionTarget sp;
    if (avoid_active) {
      // [1] AVOIDANCE: forward the held escape target verbatim (it is already a
      //     complete position+yaw setpoint in the mavros-local ENU frame),
      //     restamped, at the full 100 Hz loop rate so the OFFBOARD stream
      //     never starves (the avoidance node only emits at ~40 Hz).
      sp = last_avoid_;
      sp.header.stamp = now;
      ROS_WARN_THROTTLE(1.0, "[px4_bridge] AVOIDANCE ACTIVE -> forwarding /target_avoidance");

    } else if (enable_avoidance_ && flag_fresh && avoid_flag_ == 1 && !avoid_fresh) {
      // [2] SAFETY: flag says "avoid" but the escape target is stale. Do NOT
      //     trust EPIC (we may be right next to an obstacle) -> hold pose.
      Eigen::Vector3d p(odom_.pose.pose.position.x,
                        odom_.pose.pose.position.y,
                        odom_.pose.pose.position.z);
      const auto& q = odom_.pose.pose.orientation;
      double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      sp = makeSetpoint(p, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                        yaw, 0.0, /*full_state=*/false);
      ROS_WARN_THROTTLE(0.5, "[px4_bridge] flag=1 but /target_avoidance STALE -> hold pose");

    } else if (cmd_fresh) {
      // [3] NORMAL: track EPIC's /position_cmd (unchanged behaviour).
      Eigen::Vector3d p(last_cmd_.position.x, last_cmd_.position.y, last_cmd_.position.z);
      Eigen::Vector3d v(last_cmd_.velocity.x, last_cmd_.velocity.y, last_cmd_.velocity.z);
      Eigen::Vector3d a(last_cmd_.acceleration.x, last_cmd_.acceleration.y, last_cmd_.acceleration.z);
      sp = makeSetpoint(p, v, a, last_cmd_.yaw, last_cmd_.yaw_dot, /*full_state=*/true);

    } else {
      // [4] HOLD: pre-takeoff bootstrap, or planner-stale fallback.
      Eigen::Vector3d p(odom_.pose.pose.position.x,
                        odom_.pose.pose.position.y,
                        odom_.pose.pose.position.z);
      const auto& q = odom_.pose.pose.orientation;
      double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z));

      sp = makeSetpoint(p, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                        yaw, 0.0, /*full_state=*/false);
      if (have_cmd_)
        ROS_WARN_THROTTLE(1.0, "[px4_bridge] /position_cmd stale -> holding pose");
    }

    sp_pub_.publish(sp);
    rate.sleep();
  }
  return 0;
}
