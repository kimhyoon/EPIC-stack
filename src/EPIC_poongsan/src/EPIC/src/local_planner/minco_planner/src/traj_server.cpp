#include <gcopter/trajectory.hpp>
#include <misc/visualizer.hpp>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <traj_utils/PolyTraj.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <chrono>
#include <ctime>
using namespace Eigen;
using namespace std;
double replan_time_;
ros::Publisher pos_cmd_pub, cmd_vis_pub, traj_pub;
std::shared_ptr<Trajectory<7>> traj_;
std::shared_ptr<Trajectory<5>> yaw_traj_;
quadrotor_msgs::PositionCommand cmd;
std::vector<Eigen::Vector3d> traj_cmd_, traj_real_;
Eigen::Vector3d real_pos_;
double slowly_flip_yaw_target_, slowly_turn_to_center_target_;

bool receive_traj_ = false;
double traj_duration_, t_stop, yaw_traj_duration_, yaw_t_stop;
ros::Time start_time_;
ros::Time emergency_stop_time;
int traj_id_;
ros::Time heartbeat_time_(0);
Eigen::Vector3d last_pos_;

double last_yaw_, last_yawdot_;

// RTH mode detection
bool is_rth_mode_ = false;
std::string current_state_ = "";
size_t rth_start_index_ = 0;  // Index in traj_cmd_ where RTH starts
bool rth_index_saved_ = false;  // Flag to ensure we only save index once

// Metrics tracking
ros::Time mission_start_time_;
bool mission_started_ = false;
std::vector<double> velocity_samples_;
std::vector<double> tracking_errors_;
Eigen::Vector3d last_odom_pos_;
Eigen::Vector3d last_odom_vel_;
ros::Time last_odom_time_;
bool has_last_odom_pos_ = false;
int error_sample_count_ = 0;

std::pair<double, double> get_yaw(double current_yaw, double t_cur) {
  std::pair<double, double> yaw_yawdot(0, 0);
  if (t_cur > yaw_traj_->getTotalDuration())
    t_cur = yaw_traj_->getTotalDuration();
  Eigen::Vector3d ap = yaw_traj_->getPos(t_cur);
  double next_yaw = ap.x();
  double d_yaw = next_yaw - current_yaw;
  if (d_yaw >= M_PI) {
    d_yaw -= 2 * M_PI;
  }
  if (d_yaw <= -M_PI) {
    d_yaw += 2 * M_PI;
  }
  next_yaw = current_yaw + d_yaw;
  yaw_yawdot.first = next_yaw;
  Eigen::Vector3d av = yaw_traj_->getVel(t_cur);
  yaw_yawdot.second = av(0);
  return yaw_yawdot;
}

void stateCallback(const visualization_msgs::Marker::ConstPtr& msg) {
  // Extract state from marker text
  if (!msg->text.empty()) {
    current_state_ = msg->text;
    bool was_rth = is_rth_mode_;

    // Check if in RTH mode - look for RTH in the state string
    is_rth_mode_ = (current_state_.find("PLAN_TRAJ_RTH") != std::string::npos ||
                    current_state_.find("_RTH") != std::string::npos);

    // Start mission timer on first trajectory
    if (!mission_started_ && current_state_.find("EXEC_TRAJ") != std::string::npos) {
      mission_started_ = true;
      mission_start_time_ = ros::Time::now();
      ROS_INFO("\033[32m[Metrics] Mission started\033[0m");
    }

    // Save RTH start index when RTH mode begins (only once, only if we have some trajectory)
    if (!rth_index_saved_ && is_rth_mode_ && traj_cmd_.size() > 0) {
      rth_start_index_ = traj_cmd_.size();
      rth_index_saved_ = true;
      ROS_INFO("\033[33m[Metrics] RTH mode activated at index %zu (current state: %s)\033[0m",
               rth_start_index_, current_state_.c_str());
    }

    // Debug: print current state. FSM의 이벤트 로거(STATE 채널)가 전이를 이미
    // 기록하므로 여기는 DEBUG 로 강등 (예전엔 리플랜 플래핑마다 INFO 스팸).
    static std::string last_state;
    if (current_state_ != last_state) {
      ROS_DEBUG("[traj_server] State changed: %s (is_rth: %d, traj_size: %zu)",
                current_state_.c_str(), is_rth_mode_, traj_cmd_.size());
      last_state = current_state_;
    }
  }
}

void heartbeatCallback(std_msgs::EmptyPtr msg) {
  double fsm_time_cost = (ros::Time::now() - heartbeat_time_).toSec();
  heartbeat_time_ = ros::Time::now();
  static int count;
  static int last_count;
  if (count > 10000)
    count = 0;
  else
    count++;
  if (fsm_time_cost > 0.1) {
    std::cout << count - last_count << "\033[31m [FSM_Spin Time Cost] " << fsm_time_cost
              << "s \033[0m" << std::endl;
    last_count = count;
  }
}

void drawCmd(const Eigen::Vector3d &pos, const Eigen::Vector3d &vec, const int &id,
             const Eigen::Vector4d &color) {
  visualization_msgs::Marker mk_state;
  mk_state.header.frame_id = "odom";
  mk_state.header.stamp = ros::Time::now();
  mk_state.id = id;
  mk_state.type = visualization_msgs::Marker::ARROW;
  mk_state.action = visualization_msgs::Marker::ADD;

  mk_state.pose.orientation.w = 1.0;
  mk_state.scale.x = 0.1;
  mk_state.scale.y = 0.2;
  mk_state.scale.z = 0.3;

  geometry_msgs::Point pt;
  pt.x = pos(0);
  pt.y = pos(1);
  pt.z = pos(2);
  mk_state.points.push_back(pt);

  pt.x = pos(0) + vec(0);
  pt.y = pos(1) + vec(1);
  pt.z = pos(2) + vec(2);
  mk_state.points.push_back(pt);

  mk_state.color.r = color(0);
  mk_state.color.g = color(1);
  mk_state.color.b = color(2);
  mk_state.color.a = color(3);

  cmd_vis_pub.publish(mk_state);
}

void polyTrajCallback(traj_utils::PolyTrajPtr msg) {
  if (msg->order != 7) {
    ROS_ERROR("[traj_server] Only support trajectory order equals 7 now!");
    return;
  }
  if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size()) {
    ROS_ERROR("[traj_server] WRONG trajectory parameters, ");
    return;
  }

  int piece_nums = msg->duration.size();
  std::vector<double> dura(piece_nums);
  std::vector<Piece<7>::CoefficientMat> cMats(piece_nums);
  for (int i = 0; i < piece_nums; ++i) {
    int i6 = i * 8;
    cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
    msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5], msg->coef_x[i6 + 6],
    msg->coef_x[i6 + 7];
    cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
    msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5], msg->coef_y[i6 + 6],
    msg->coef_y[i6 + 7];
    cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
    msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5], msg->coef_z[i6 + 6],
    msg->coef_z[i6 + 7];

    dura[i] = msg->duration[i];
  }

  traj_.reset(new Trajectory<7>(dura, cMats));

  start_time_ = msg->start_time;
  traj_duration_ = traj_->getTotalDuration();
  t_stop = traj_duration_;
  traj_id_ = msg->traj_id;

  receive_traj_ = true;
  std::vector<Eigen::Vector3d> a;
}

void polyYawTrajCallback(traj_utils::PolyTrajPtr msg) {
  if (msg->order != 5) {
    ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
    return;
  }
  if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size()) {
    ROS_ERROR("[traj_server] WRONG trajectory parameters, ");
    return;
  }
  // std::cout << "receive yaw traj" << yaw_t_stop + start_time_.toSec() - msg->start_time.toSec()
  // << "s to go" << std::endl;

  int piece_nums = msg->duration.size();
  std::vector<double> dura(piece_nums);
  std::vector<Piece<5>::CoefficientMat> cMats(piece_nums);
  for (int i = 0; i < piece_nums; ++i) {
    int i6 = i * 6;
    cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
    msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
    cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
    msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
    cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
    msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

    dura[i] = msg->duration[i];
  }
  yaw_traj_.reset(new Trajectory<5>(dura, cMats));
  yaw_traj_duration_ = yaw_traj_->getTotalDuration();
  yaw_t_stop = yaw_traj_duration_;
}

void publish_cmd(Vector3d p, Vector3d v, Vector3d a, Vector3d j, double y, double yd) {
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "odom";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = p(0);
  cmd.position.y = p(1);
  cmd.position.z = p(2);
  cmd.velocity.x = v(0);
  cmd.velocity.y = v(1);
  cmd.velocity.z = v(2);
  cmd.acceleration.x = a(0);
  cmd.acceleration.y = a(1);
  cmd.acceleration.z = a(2);
  cmd.jerk.x = j(0);
  cmd.jerk.y = j(1);
  cmd.jerk.z = j(2);
  cmd.yaw = y;
  cmd.yaw_dot = yd;
  pos_cmd_pub.publish(cmd);

  last_pos_ = p;
}

void cmdCallback(const ros::TimerEvent &e) {
  /* no publishing before receive traj_ and have heartbeat */
  if (heartbeat_time_.toSec() <= 1e-5) {
    // ROS_ERROR_ONCE("[traj_server] No heartbeat from the planner received");
    return;
  }
  if (!receive_traj_)
    return;

  ros::Time time_now = ros::Time::now();
  static bool printed;
  if ((time_now - heartbeat_time_).toSec() > 0.5) {
    if (!printed) {
      ROS_ERROR("[traj_server] Lost heartbeat from the planner, is it dead?");
      printed = true;
    }
  } else {
    printed = false;
  }

  double t_cur = (time_now - start_time_).toSec();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()),
  acc(Eigen::Vector3d::Zero()), jer(Eigen::Vector3d::Zero());
  std::pair<double, double> yaw_yawdot(0, 0);

  static ros::Time time_last = ros::Time::now();

  // double traj_dur = traj_duration_;

  // if (traj_duration_ < yaw_traj_duration_)
  //   traj_dur = yaw_traj_duration_;

  // if (t_cur < traj_dur && t_cur >= 0.0) {
  if (t_cur < traj_duration_) {
    pos = traj_->getPos(t_cur);
    vel = traj_->getVel(t_cur);
    acc = traj_->getAcc(t_cur);
    jer = traj_->getJer(t_cur);

  } else {
    pos = last_pos_;
    vel = Eigen::Vector3d::Zero();
    acc = Eigen::Vector3d::Zero();
    jer = Eigen::Vector3d::Zero();
  }

  if (t_cur < yaw_traj_duration_) {
    yaw_yawdot = get_yaw(last_yaw_, t_cur);
  } else {
    yaw_yawdot = std::make_pair(last_yaw_, 0.0);
  }

  time_last = time_now;
  last_yaw_ = yaw_yawdot.first;
  last_pos_ = pos;

  publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
  if (traj_cmd_.size() == 0)
    traj_cmd_.emplace_back(pos);
  else if ((traj_cmd_.back() - pos).norm() > 0.02)
    traj_cmd_.emplace_back(pos);

  // Calculate tracking error (planned vs actual position)
  if (mission_started_ && real_pos_.norm() > 0.01) {
    double tracking_error = (pos - real_pos_).norm();
    tracking_errors_.push_back(tracking_error);
    error_sample_count_++;
  }

  // if (traj_cmd_.size() > 10000)
  //   traj_cmd_.erase(traj_cmd_.begin(), traj_cmd_.begin() + 1000);
  drawCmd(pos, vel, 0, Eigen::Vector4d(0, 1, 0, 1));
  // }
}

void odomCallbck(const nav_msgs::Odometry &msg) {
  if (msg.child_frame_id == "X" || msg.child_frame_id == "O")
    return;

  Eigen::Vector3d current_pos(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);

  if (traj_real_.size() == 0) {
    traj_real_.push_back(current_pos);
  } else if ((traj_real_.back() - current_pos).norm() > 0.1) {
    traj_real_.emplace_back(current_pos);
  }

  real_pos_ = current_pos;

  if (traj_real_.size() > 100000)
    traj_real_.erase(traj_real_.begin(), traj_real_.begin() + 1000);

  // Sample velocity for average speed calculation
  if (mission_started_) {
    ros::Time current_time = ros::Time::now();

    // Calculate velocity from position difference if we have previous position
    if (has_last_odom_pos_) {
      double dt = (current_time - last_odom_time_).toSec();
      if (dt > 0.001) {  // Avoid division by very small numbers
        Eigen::Vector3d displacement = current_pos - last_odom_pos_;
        double speed = displacement.norm() / dt;
        if (speed > 0.01 && speed < 10.0) {  // Filter noise and unrealistic speeds
          velocity_samples_.push_back(speed);
        }
      }
    }

    last_odom_pos_ = current_pos;
    last_odom_time_ = current_time;
    has_last_odom_pos_ = true;
  }
}

void displayTrajWithColor(std::vector<Eigen::Vector3d> &path, double resolution,
                          Eigen::Vector4d color, int id) {
  visualization_msgs::MarkerArray mk_arr;
  visualization_msgs::Marker mk;
  mk.header.frame_id = "odom";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;

  // Set namespace based on id
  if (id == 0) {
    mk.ns = "traj_exp";
    mk.action = visualization_msgs::Marker::DELETEALL;
    mk.id = id;
    mk_arr.markers.emplace_back(mk);
  } else if (id == 1) {
    mk.ns = "traj_real";
  } else if (id == 2) {
    mk.ns = "traj_rth";  // Separate namespace for RTH trajectory
  }

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;
  geometry_msgs::Point pt;
  for (int i = 0; i < int(path.size()); i++) {
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    mk.points.push_back(pt);
  }
  mk_arr.markers.push_back(mk);
  traj_pub.publish(mk_arr);
  ros::Duration(0.001).sleep();
}

void visCallback(const ros::TimerEvent &e) {
  // Display exploration and RTH paths with different colors
  if (rth_start_index_ > 0 && rth_start_index_ < traj_cmd_.size()) {
    // Split trajectory: exploration (white) and RTH (orange)
    std::vector<Eigen::Vector3d> exploration_path(traj_cmd_.begin(), traj_cmd_.begin() + rth_start_index_);
    std::vector<Eigen::Vector3d> rth_path(traj_cmd_.begin() + rth_start_index_, traj_cmd_.end());

    displayTrajWithColor(exploration_path, 0.08, Eigen::Vector4d(1, 1, 1, 1), 0);  // White
    displayTrajWithColor(rth_path, 0.08, Eigen::Vector4d(1, 0.5, 0, 1), 2);  // Orange, different id
  } else {
    // No RTH yet, display all as exploration (white)
    displayTrajWithColor(traj_cmd_, 0.08, Eigen::Vector4d(1, 1, 1, 1), 0);
  }

  displayTrajWithColor(traj_real_, 0.08, Eigen::Vector4d(0, 0, 1, 1), 1);
}

void replanCallback(const std_msgs::Empty &msg) {
  const double time_out = 0.1;
  ros::Time time_now = ros::Time::now();
  double tmp = (time_now - start_time_).toSec() + replan_time_ + time_out;
  traj_duration_ = std::min(traj_duration_, tmp);
  // yaw_traj_duration_ = std::min(yaw_traj_duration_, tmp);
  // std::cout << "\033[32m [TrajServer] Replan , stop after "
  //           << traj_duration_ - (time_now - start_time_).toSec() << "s \033[0m"
  //           << std::endl;
}

double calculatePathLength(const std::vector<Eigen::Vector3d>& path) {
  double total_length = 0.0;
  for (size_t i = 1; i < path.size(); i++) {
    total_length += (path[i] - path[i-1]).norm();
  }
  return total_length;
}

double calculateAverageSpeed() {
  if (velocity_samples_.empty()) return 0.0;
  double sum = 0.0;
  for (double v : velocity_samples_) {
    sum += v;
  }
  return sum / velocity_samples_.size();
}

double calculateAverageTrackingError() {
  if (tracking_errors_.empty()) return 0.0;
  double sum = 0.0;
  for (double e : tracking_errors_) {
    sum += e;
  }
  return sum / tracking_errors_.size();
}

void logMetrics(double rth_distance_to_goal) {
  if (!mission_started_) return;

  double mission_time = (ros::Time::now() - mission_start_time_).toSec();
  double avg_speed_odom = calculateAverageSpeed();
  double path_length = calculatePathLength(traj_cmd_);
  double avg_speed_path = (mission_time > 0.0) ? (path_length / mission_time) : 0.0;
  double avg_tracking_error = calculateAverageTrackingError();

  // Log to file
  std::string log_file = std::string(getenv("HOME")) + "/.ros/epic_metrics.log";
  std::ofstream ofs(log_file, std::ios::app);

  if (ofs.is_open()) {
    // Write header if file is new
    std::ifstream check_file(log_file);
    check_file.seekg(0, std::ios::end);
    bool is_new_file = (check_file.tellg() == 0);
    check_file.close();

    if (is_new_file) {
      ofs << "timestamp,mission_time(s),rth_distance_to_goal(m),avg_speed_odom(m/s),path_length(m),avg_speed_path(m/s),avg_tracking_error(m),velocity_samples,error_samples\n";
    }

    // Get current timestamp
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    char timestamp[100];
    std::strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", std::localtime(&now_time_t));

    ofs << timestamp << ","
        << mission_time << ","
        << rth_distance_to_goal << ","
        << avg_speed_odom << ","
        << path_length << ","
        << avg_speed_path << ","
        << avg_tracking_error << ","
        << velocity_samples_.size() << ","
        << tracking_errors_.size() << "\n";

    ofs.close();

    ROS_INFO("\033[32m[Metrics] Mission Complete - Logged to %s\033[0m", log_file.c_str());
    ROS_INFO("\033[36m  Mission time: %.2f s\033[0m", mission_time);
    ROS_INFO("\033[36m  RTH distance to goal: %.3f m\033[0m", rth_distance_to_goal);
    ROS_INFO("\033[36m  Avg speed (odometry): %.3f m/s\033[0m", avg_speed_odom);
    ROS_INFO("\033[36m  Path length: %.2f m\033[0m", path_length);
    ROS_INFO("\033[36m  Avg speed (path/time): %.3f m/s\033[0m", avg_speed_path);
    ROS_INFO("\033[36m  Avg tracking error: %.3f m\033[0m", avg_tracking_error);
  } else {
    ROS_ERROR("[Metrics] Failed to open log file: %s", log_file.c_str());
  }
}

void rthDistanceCallback(const std_msgs::Float32::ConstPtr& msg) {
  // RTH completed, log all metrics
  double rth_distance = msg->data;
  ROS_INFO("\033[33m[Metrics] RTH distance received: %.3f m, logging metrics...\033[0m", rth_distance);
  logMetrics(rth_distance);
}

void newCallback(std_msgs::Empty msg) {
  // Clear the executed traj data
  traj_cmd_.clear();
  traj_real_.clear();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "traj_server");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~");
  // ros::Subscriber emergency_sub =
  //     nh.subscribe("/planning/emergency_stop", 10, emergencyStopCb);
  // odom 토픽은 config yaml 의 odometry_topic (exploration_node ns) 이 유일한
  // 소스다. 폴백 절대 금지 — 잘못된 토픽 구독은 실비행 추락으로 이어지므로
  // 파라미터가 없으면 시작을 거부한다.
  std::string odom_topic;
  if (!ros::param::get("/exploration_node/odometry_topic", odom_topic) ||
      odom_topic.empty()) {
    ROS_FATAL("[Traj server] /exploration_node/odometry_topic not set "
              "(config yaml not loaded?). REFUSING TO START - no fallback.");
    return 1;
  }
  ROS_INFO("[Traj server]: odom topic: %s", odom_topic.c_str());

  ros::Subscriber poly_traj_sub = nh.subscribe("/planning/trajectory", 10, polyTrajCallback);
  ros::Subscriber poly_yaw_traj_sub =
  nh.subscribe("/planning/yaw_trajectory", 10, polyYawTrajCallback);
  ros::Subscriber heartbeat_sub = nh.subscribe("/planning/heartbeat", 10, heartbeatCallback);
  ros::Subscriber odom_sub = nh.subscribe(odom_topic, 50, odomCallbck);
  ros::Subscriber replan_sub = nh.subscribe("/planning/replan", 10, replanCallback);
  ros::Subscriber new_sub = nh.subscribe("planning/new", 10, newCallback);
  ros::Subscriber state_sub = nh.subscribe("/planning/state", 10, stateCallback);
  ros::Subscriber rth_dist_sub = nh.subscribe("/planning/rth_distance", 10, rthDistanceCallback);

  nh.param("/fsm/replan_time", replan_time_, 0.1);
  ros::Timer vis_timer = nh.createTimer(ros::Duration(0.25), visCallback);
  pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  cmd_vis_pub = nh.advertise<visualization_msgs::Marker>("/planning/position_cmd_vis", 10);
  traj_pub = nh.advertise<visualization_msgs::MarkerArray>("/planning/travel_traj", 10);

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

  last_yaw_ = 0.0;
  last_yawdot_ = 0.0;

  ros::Duration(1.0).sleep();

  ROS_INFO("[Traj server]: ready.");

  ros::spin();

  return 0;
}