/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2024-02-29 16:54:46
 * @LastEditTime: 2024-03-11 13:22:44
 * @Description:
 * @
 * @Copyright (c) 2024 by ning-zelin, All Rights Reserved.
 */

#include <epic_planner/expl_data.h>
#include <epic_planner/fast_exploration_fsm.h>
#include <epic_planner/fast_exploration_manager.h>
#include <plan_manage/planner_manager.h>
#include <cmath>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <traj_utils/planning_visualization.h>
using Eigen::Vector3d;
using Eigen::Vector4d;
bool debug_planner;
typedef visualization_msgs::Marker Marker;
typedef visualization_msgs::MarkerArray MarkerArray;

void FastExplorationFSM::FSMCallback(const ros::TimerEvent &e) {
  pubState();

  // ---- reactive-avoidance hand-off (Phase 2) ------------------------------
  // The px4_ctrl_bridge MUX overrides EPIC's command with the reactive escape
  // setpoint while /FSM_flag_avoidance==1, so the drone leaves EPIC's planned
  // path. EPIC keeps planning throughout (this hook never stops it), but we
  // force every replan to anchor to the drone's ACTUAL pose (static_state_)
  // instead of a predicted point on the old trajectory. On release we force one
  // fresh replan so the trajectory handed back to PX4 starts where the drone
  // actually is -> no snap-back toward the obstacle.
  const bool avoiding =
      have_avoid_flag_ && (avoid_flag_ == 1) &&
      ((ros::Time::now() - last_avoid_flag_stamp_).toSec() < avoid_flag_timeout_);
  const bool mission_active =
      (state_ == EXEC_TRAJ || state_ == PLAN_TRAJ_EXP || state_ == PLAN_TRAJ_RTH);
  if (avoiding && mission_active)
    fd_->static_state_ = true;
  if (avoiding_prev_ && !avoiding && fd_->trigger_ && mission_active) {
    fd_->static_state_ = true;
    EXPL_STATE next_state = has_goal_rth_ ? PLAN_TRAJ_RTH : PLAN_TRAJ_EXP;
    transitState(next_state, "avoidance released: replan from current pose", true);
  }
  avoiding_prev_ = avoiding;

  switch (state_) {
  case INIT: {
    if (!fd_->have_odom_) {
      ROS_WARN_THROTTLE(1.0, "no odom.");
      return;
    }
    transitState(WAIT_TRIGGER, "FSM");
    break;
  }

  case WAIT_TRIGGER: {
    ROS_WARN_THROTTLE(1.0, "wait for trigger.");
    break;
  }

  case TAKEOFF_HOVER: {
    // Triggered -> climb to the configured altitude and hold, then auto-start
    // exploration once odom confirms the drone is stable near that altitude.
    if (!fd_->have_odom_)
      return;

    // Stream the hover setpoint (hold x,y,yaw; target altitude) at the FSM rate so
    // px4_ctrl_bridge keeps it "fresh" and forwards it to PX4.
    pubHoverCmd();

    double z_err = std::fabs((double)fd_->odom_pos_.z() - takeoff_anchor_.z());
    double speed = fd_->odom_vel_.norm();
    bool reached = (z_err < fp_->takeoff_reach_tol_) && (speed < fp_->takeoff_settle_vel_);

    ros::Time now = ros::Time::now();
    if (reached) {
      if (hover_stable_since_.toSec() < 1e-6)
        hover_stable_since_ = now;  // start the settle timer
      if ((now - hover_stable_since_).toSec() >= fp_->takeoff_settle_time_) {
        fd_->static_state_ = true;  // first exploration traj anchors to current pose
        transitState(PLAN_TRAJ_EXP,
                     "takeoff: altitude reached & stable -> explore");
        break;
      }
    } else {
      hover_stable_since_ = ros::Time(0);  // not stable -> reset settle timer
    }

    // Safety: never wait forever -- but never start exploration from a non-airborne
    // pose either. If we time out while roughly at altitude (allow up to 3x the reach
    // tolerance to absorb odom/LIO z drift) and not climbing fast, proceed. Otherwise the
    // climb genuinely failed (not armed / not OFFBOARD, thrust-limited stall, bad z odom)
    // -> keep holding the climb setpoint and shout, rather than commanding lateral motion
    // from the ground.
    if ((now - hover_enter_time_).toSec() > fp_->takeoff_timeout_) {
      const double relaxed_tol = 3.0 * fp_->takeoff_reach_tol_;
      if (z_err < relaxed_tol && speed < 2.0 * fp_->takeoff_settle_vel_) {
        ROS_WARN("[takeoff] timeout %.1fs, near altitude (z_err=%.2f m, v=%.2f m/s) -> explore",
                 fp_->takeoff_timeout_, z_err, speed);
        fd_->static_state_ = true;
        transitState(PLAN_TRAJ_EXP, "takeoff: timeout (near altitude) -> explore");
      } else {
        ROS_ERROR_THROTTLE(2.0,
            "[takeoff] timeout %.1fs and NOT at altitude (z_err=%.2f m, v=%.2f m/s) -> "
            "holding hover, NOT exploring (check arming / OFFBOARD / thrust)",
            fp_->takeoff_timeout_, z_err, speed);
        // stay in TAKEOFF_HOVER; pubHoverCmd() keeps streaming the climb setpoint.
      }
    }
    break;
  }

  case FINISH: {
    // stopTraj();
    double collision_time = 0.0;
    bool safe = planner_manager_->checkTrajCollision(collision_time);
    if (!safe) {
      stopTraj();
    }
    ROS_WARN_THROTTLE(1.0, "Finished.");
    break;
  }

  case PLAN_TRAJ_EXP: {
    if (!fd_->trigger_)
      return;
    if (planner_manager_->topo_graph_->odom_node_->neighbors_.empty())
      return;
    ros::Time start = ros::Time::now();
    // 要报min-step的case
    LocalTrajData *info = &planner_manager_->local_data_;
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    double time_to_end = info->duration_ - t_cur;
    if (expl_manager_->ed_->global_tour_.size() == 2) {
      Eigen::Vector3f goal = expl_manager_->ed_->global_tour_[1];
      if ((goal - fd_->odom_pos_).norm() < 1e-1) {
        transitState(FINISH, "fsm");
        return;
      }
    }
    ros::Time tplan = ros::Time::now();
    exec_timer_.stop();
    int res = callExplorationPlanner();
    exec_timer_.start();
    ROS_INFO("\033[31m call planner \033[0m: %.3f",
             (ros::Time::now() - tplan).toSec() * 1000.0);

    if (res == SUCCEED) {
      poly_yaw_traj_pub_.publish(fd_->newest_yaw_traj_);
      poly_traj_pub_.publish(fd_->newest_traj_);
      fd_->static_state_ = false;
      if (fd_->use_bubble_a_star_) {
        transitState(EXEC_TRAJ,
                     "ParallelBubbleAstar plan success: new traj pub");
      } else {
        transitState(EXEC_TRAJ, "plan success: new traj pub");
      }
      fd_->use_bubble_a_star_ = false;
      fd_->half_resolution = false;

    } else if (res == NO_FRONTIER) {
      // if (planner_manager_->topo_graph_->global_view_points_.empty())
      transitState(FINISH, "PLAN_TRAJ_EXP: no frontier");
      fd_->static_state_ = true;
    } else if (res == FAIL) {
      // Still in PLAN_TRAJ_EXP state, keep replanning
      stopTraj();
      transitState(PLAN_TRAJ_EXP, "PLAN_TRAJ_EXP: plan failed", true);

    } else if (res == START_FAIL) {
      transitState(CAUTION, "PLAN_TRAJ_EXP: start failed", true);
    } else {
      cout << "330?" << endl;
    }
    break;
  }

  case PLAN_TRAJ_RTH: {
    if (!has_goal_rth_)
      return;
    if (planner_manager_->topo_graph_->odom_node_->neighbors_.empty())
      return;

    // Check if goal reached
    double dist = (fd_->odom_pos_.cast<double>() - goal_rth_.head<3>()).norm();
    ROS_INFO("\033[36m[RTH] Distance to goal: %.3f m (tolerance: %.3f m)\033[0m", dist, goal_tolerance_);

    if (dist < goal_tolerance_) {
      has_goal_rth_ = false;
      global_path_update_timer_.stop();  // Stop replanning timer

      // Publish RTH distance for metrics logging
      std_msgs::Float32 dist_msg;
      dist_msg.data = dist;
      rth_metrics_pub_.publish(dist_msg);

      transitState(FINISH, "PLAN_TRAJ_RTH: goal reached");
      ROS_INFO("\033[32m[RTH] Goal reached! \033[0m");
      return;
    }

    ros::Time tplan = ros::Time::now();
    exec_timer_.stop();
    int res = callGoalPlanner();
    exec_timer_.start();
    ROS_INFO("\033[31m call goal planner \033[0m: %.3f",
             (ros::Time::now() - tplan).toSec() * 1000.0);

    if (res == SUCCEED) {
      poly_yaw_traj_pub_.publish(fd_->newest_yaw_traj_);
      poly_traj_pub_.publish(fd_->newest_traj_);
      fd_->static_state_ = false;
      transitState(EXEC_TRAJ, "PLAN_TRAJ_RTH: plan success");
      fd_->use_bubble_a_star_ = false;
      fd_->half_resolution = false;

    } else if (res == FAIL) {
      // Still in PLAN_TRAJ_RTH state, keep replanning
      stopTraj();
      transitState(PLAN_TRAJ_RTH, "PLAN_TRAJ_RTH: plan failed", true);

    } else if (res == START_FAIL) {
      transitState(CAUTION, "PLAN_TRAJ_RTH: start failed", true);
    }
    break;
  }

  case EXEC_TRAJ: {
    // collision check
    double collision_time;
    bool safe = planner_manager_->checkTrajCollision(collision_time);
    if (!safe) {
      // Return to appropriate planning state
      EXPL_STATE next_state = has_goal_rth_ ? PLAN_TRAJ_RTH : PLAN_TRAJ_EXP;
      transitState(
          next_state,
          "safetyCallback: not safe, time:" + to_string(collision_time), true);
      if (collision_time < fp_->replan_time_ + 0.2)
        stopTraj();
    } else if (!planner_manager_->checkTrajVelocity()) {
      EXPL_STATE next_state = has_goal_rth_ ? PLAN_TRAJ_RTH : PLAN_TRAJ_EXP;
      transitState(next_state, "velocity too fast", true);
    } else {
      // Emergency control-error replan: continuous replan anchors the next traj to a
      // predicted point on the OLD trajectory (it never looks at the actual pose). If
      // the drone has drifted too far from the traj it is tracking (wind, controller
      // saturation, FAST-LIO pose jump), that assumption is broken -> force a replan
      // anchored to the current pose (static_state_=true). <= 0 disables the check.
      // Skip while reactive avoidance is active: the drone deliberately leaves the
      // planned path then, and the avoidance hook already forces a from-current-pose
      // replan -- firing here would only spam redundant replans against a path PX4 isn't
      // even tracking. (`avoiding` is computed at the top of FSMCallback.)
      LocalTrajData *info = &planner_manager_->local_data_;
      if (!avoiding && info->traj_id_ > 1 && fp_->emergency_replan_control_error > 0.0) {
        double t_cur = (ros::Time::now() - info->start_time_).toSec();
        if (t_cur >= 0.0 && t_cur <= info->duration_) {
          Eigen::Vector3d planned = info->minco_traj_.getPos(t_cur);
          double ctrl_err = (fd_->odom_pos_.cast<double>() - planned).norm();
          if (ctrl_err > fp_->emergency_replan_control_error) {
            ROS_WARN("\033[31m[EMERGENCY] control error %.2f m > %.2f m -> replan from "
                     "current pose\033[0m",
                     ctrl_err, fp_->emergency_replan_control_error);
            fd_->static_state_ = true;
            EXPL_STATE next_state = has_goal_rth_ ? PLAN_TRAJ_RTH : PLAN_TRAJ_EXP;
            transitState(next_state, "emergency: control error", true);
            stopTraj();
          }
        }
      }
    }

    break;
  }

  case CAUTION: {
    stopTraj();
    exec_timer_.stop();
    bool success = planner_manager_->flyToSafeRegion(fd_->static_state_);
    if (success) {
      traj_utils::PolyTraj poly_traj_msg;
      auto info = &planner_manager_->local_data_;
      planner_manager_->polyTraj2ROSMsg(poly_traj_msg, info->start_time_);
      fd_->newest_traj_ = poly_traj_msg;
      poly_traj_pub_.publish(fd_->newest_traj_);
      ros::Duration(0.2).sleep();
    }
    exec_timer_.start();
    double dis2occ =
        planner_manager_->lidar_map_interface_->getDisToOcc(fd_->odom_pos_);
    if (dis2occ > planner_manager_->gcopter_config_->dilateRadiusSoft) {
      EXPL_STATE next_state = has_goal_rth_ ? PLAN_TRAJ_RTH : PLAN_TRAJ_EXP;
      transitState(next_state, "safe now");
    }
    break;
  }
  case LAND: {
    stopTraj();
    exec_timer_.stop();
    global_path_update_timer_.stop();
    // 没电了！！再飞就会炸鸡，降落！！！
    while (1) {
      quadrotor_msgs::TakeoffLand land_msg;
      land_msg.takeoff_land_cmd = land_msg.LAND;
      land_pub_.publish(land_msg);
      ros::Duration(0.2).sleep();
      ROS_WARN_THROTTLE(1.0, "NO POWER. LAND!!");
    }

    break;
  }
  }
}

void FastExplorationFSM::pubHoverCmd() {
  // Hold (x0, y0, target_z) with the heading captured at trigger time. px4_ctrl_bridge
  // forwards this as a position setpoint (it ignores the velocity field), so the drone
  // climbs to target_z and hovers. Mirrors traj_server's /position_cmd convention.
  quadrotor_msgs::PositionCommand cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "odom";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = 0;
  cmd.position.x = takeoff_anchor_.x();
  cmd.position.y = takeoff_anchor_.y();
  cmd.position.z = takeoff_anchor_.z();
  cmd.velocity.x = cmd.velocity.y = cmd.velocity.z = 0.0;
  cmd.acceleration.x = cmd.acceleration.y = cmd.acceleration.z = 0.0;
  cmd.jerk.x = cmd.jerk.y = cmd.jerk.z = 0.0;
  cmd.yaw = takeoff_yaw_;
  cmd.yaw_dot = 0.0;
  hover_cmd_pub_.publish(cmd);
}

void FastExplorationFSM::init(ros::NodeHandle &nh,
                              FastExplorationManager::Ptr &explorer) {
  fp_.reset(new FSMParam);
  fd_.reset(new FSMData);

  /*  Fsm param  */
  nh.param("fsm/thresh_replan", fp_->replan_thresh_, -1.0);
  nh.param("fsm/replan_time", fp_->replan_time_, -1.0);
  nh.param("bubble_astar/resolution_astar", fp_->bubble_a_star_resolution, 0.1);
  nh.param("fsm/debug_planner", debug_planner, false);
  // Default 1.5 matches the value previously hardcoded in algorithm.xml, so configs
  // that don't set this key keep their old (now-active) behaviour. real.yaml overrides it.
  nh.param("fsm/emergency_replan_control_error",
           fp_->emergency_replan_control_error, 1.5);
  // takeoff & hover-before-explore (see config yaml). Default DISABLED (<= 0): only
  // configs that explicitly set fsm/takeoff_height (e.g. real.yaml = 1.0) opt in, so the
  // sim configs keep the original "explore immediately on trigger" behaviour.
  nh.param("fsm/takeoff_height", fp_->takeoff_height_, -1.0);
  nh.param("fsm/takeoff_reach_tol", fp_->takeoff_reach_tol_, 0.15);
  nh.param("fsm/takeoff_settle_vel", fp_->takeoff_settle_vel_, 0.15);
  nh.param("fsm/takeoff_settle_time", fp_->takeoff_settle_time_, 1.0);
  nh.param("fsm/takeoff_timeout", fp_->takeoff_timeout_, 20.0);
  nh.param("fsm/replan_time_after_traj_start",
           fp_->replan_time_after_traj_start_, 0.5);
  nh.param("fsm/replan_time_before_traj_end", fp_->replan_time_before_traj_end_,
           0.5);
  nh.param("fsm/goal_tolerance", goal_tolerance_, 0.2);
  nh.param("fsm/avoid_flag_timeout", avoid_flag_timeout_, 0.5);
  nh.param("fsm/local_planning_max_hz", local_planning_max_hz_, 100.0);
  local_planning_min_period_ = 1.0 / local_planning_max_hz_;
  ROS_INFO("Local planning max Hz: %.1f (min period: %.4f s)", local_planning_max_hz_, local_planning_min_period_);
  /* Initialize main modules */
  // expl_manager_.reset(new FastExplorationManager);
  // expl_manager_->initialize(nh);
  expl_manager_ = explorer;
  planner_manager_ = expl_manager_->planner_manager_;

  state_ = EXPL_STATE::INIT;
  fd_->have_odom_ = false;
  fd_->state_str_ = {"INIT",      "WAIT_TRIGGER", "PLAN_TRAJ_EXP", "PLAN_TRAJ_RTH",
                     "CAUTION",   "EXEC_TRAJ",    "FINISH",        "LAND",
                     "TAKEOFF_HOVER"};
  fd_->static_state_ = true;
  fd_->trigger_ = false;
  fd_->use_bubble_a_star_ = false;
  has_goal_rth_ = false;
  battary_sub_ =
      nh.subscribe("/mavros/battery", 10, &FastExplorationFSM::battaryCallback,
                   this, ros::TransportHints().tcpNoDelay());

  /* Ros sub, pub and timer */
  // if (debug_planner) {
  //   exec_timer_ = nh.createTimer(ros::Duration(0.01),
  //   &FastExplorationFSM::PlannerDebugFSMCallback, this);
  // } else {
  exec_timer_ = nh.createTimer(ros::Duration(0.01),
                               &FastExplorationFSM::FSMCallback, this);
  // }
  global_path_update_timer_ = nh.createTimer(
      ros::Duration(0.2), &FastExplorationFSM::globalPathUpdateCallback, this);
  trigger_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1,
                              &FastExplorationFSM::triggerCallback, this);
  avoid_flag_sub_ = nh.subscribe("/FSM_flag_avoidance", 10,
                                 &FastExplorationFSM::avoidFlagCallback, this,
                                 ros::TransportHints().tcpNoDelay());
  srv_goal_ = nh.advertiseService("/srv_rth", &FastExplorationFSM::goalServiceCallback, this);
  replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);

  heartbeat_pub_ = nh.advertise<std_msgs::Empty>("/planning/heartbeat", 10);
  land_pub_ =
      nh.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/takeoff_land", 10);

  poly_traj_pub_ =
      nh.advertise<traj_utils::PolyTraj>("/planning/trajectory", 10);
  poly_yaw_traj_pub_ =
      nh.advertise<traj_utils::PolyTraj>("/planning/yaw_trajectory", 10);
  time_cost_pub_ = nh.advertise<std_msgs::Float32>("/time_cost", 10);
  static_pub_ = nh.advertise<std_msgs::Bool>("/planning/static", 10);
  state_pub_ = nh.advertise<visualization_msgs::Marker>("/planning/state", 10);
  rth_metrics_pub_ = nh.advertise<std_msgs::Float32>("/planning/rth_distance", 10);
  // Hover setpoint stream during TAKEOFF_HOVER. Absolute topic name = traj_server's
  // /position_cmd; the two never publish at the same time (traj_server is silent until
  // a trajectory exists, and we only publish here before exploration starts).
  hover_cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);

  // Global planning timing publishers
  update_topo_skeleton_cost_pub_ = nh.advertise<std_msgs::Float32>("/planning/timing/update_topo_skeleton_cost", 10);
  update_odom_vertex_cost_pub_ = nh.advertise<std_msgs::Float32>("/planning/timing/update_odom_vertex_cost", 10);
  vp_cluster_cost_pub_ = nh.advertise<std_msgs::Float32>("/planning/timing/vp_cluster_cost", 10);
  remove_unreachable_cost_pub_ = nh.advertise<std_msgs::Float32>("/planning/timing/remove_unreachable_cost", 10);
  select_vp_cost_pub_ = nh.advertise<std_msgs::Float32>("/planning/timing/select_vp_cost", 10);
  insert_viewpoint_cost_pub_ = nh.advertise<std_msgs::Float32>("/planning/timing/insert_viewpoint_cost", 10);
  calculate_tsp_cost_pub_ = nh.advertise<std_msgs::Float32>("/planning/timing/calculate_tsp_cost", 10);
  lkh_solver_cost_pub_ = nh.advertise<std_msgs::Float32>("/planning/timing/lkh_solver_cost", 10);
  call_planner_cost_pub_ = nh.advertise<std_msgs::Float32>("/planning/timing/call_planner_cost", 10);
  ikd_tree_insert_cost_pub_ = nh.advertise<std_msgs::Float32>("/planning/timing/ikd_tree_insert_cost", 10);
  update_frontier_clusters_cost_pub_ = nh.advertise<std_msgs::Float32>("/planning/timing/update_frontier_clusters_cost", 10);
  fast_searcher_search_cost_pub_ = nh.advertise<std_msgs::Float32>("/planning/timing/fast_searcher_search_cost", 10);
  bubble_astar_search_cost_pub_ = nh.advertise<std_msgs::Float32>("/planning/timing/bubble_astar_search_cost", 10);

  string odom_topic, cloud_topic;
  nh.getParam("odometry_topic", odom_topic);
  nh.getParam("cloud_topic", cloud_topic);
  cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(
      nh, cloud_topic, 1));
  odom_sub_.reset(
      new message_filters::Subscriber<nav_msgs::Odometry>(nh, odom_topic, 5));
  sync_cloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyCloudOdom>(
      SyncPolicyCloudOdom(10), *cloud_sub_, *odom_sub_));
  sync_cloud_odom_->registerCallback(
      boost::bind(&FastExplorationFSM::CloudOdomCallback, this, _1, _2));
}

void FastExplorationFSM::battaryCallback(
    const sensor_msgs::BatteryStateConstPtr &msg) {
  // if(msg->voltage < 21.0){
  //   transitState(LAND, "battary low");
  // }
}

void FastExplorationFSM::updateTopoAndGlobalPath() {
  if (!(state_ == WAIT_TRIGGER || state_ == PLAN_TRAJ_EXP || state_ == PLAN_TRAJ_RTH ||
        state_ == EXEC_TRAJ || state_ == FINISH)) {
    global_path_update_timer_.stop();
    // expl_manager_->frontier_manager_ptr_->viz_pocc();
    expl_manager_->frontier_manager_ptr_->visfrtcluster();
    global_path_update_timer_.start();
    return;
  }
  static int cnt = 0;
  cnt++;

  global_path_update_timer_.stop();
  ros::Time t2 = ros::Time::now();
  planner_manager_->topo_graph_->getRegionsToUpdate();
  // cout << "getRegionsToUpdate time cost:" << (ros::Time::now() - t2).toSec()
  // * 1000 << "ms" << endl;
  planner_manager_->topo_graph_->updateSkeleton();

  ros::Time t3 = ros::Time::now();
  planner_manager_->topo_graph_->updateOdomNode(fd_->odom_pos_, fd_->odom_yaw_);
  planner_manager_->topo_graph_->updateHistoricalOdoms();

  if (planner_manager_->topo_graph_->odom_node_->neighbors_.empty()) {
    double time;
    if (planner_manager_->local_data_.traj_id_ > 1) {
      bool safe = planner_manager_->checkTrajCollision(time);
      if (!safe) {
        transitState(CAUTION, "odom_node no nbrs");
      } else {
        global_path_update_timer_.start();

        return;
      }
    } else {
      transitState(CAUTION, "odom_node no nbrs");
    }
    global_path_update_timer_.start();
    return;
  }
  if (planner_manager_->local_data_.traj_id_ > 1) {

    double curr_time =
        (ros::Time::now() - planner_manager_->local_data_.start_time_).toSec();
    double time;
    bool safe = planner_manager_->checkTrajCollision(time);
    double total_time = planner_manager_->local_data_.duration_;
    double time2end = total_time - curr_time;

    if (safe && curr_time < fp_->replan_time_after_traj_start_ &&
        time2end > fp_->replan_time_before_traj_end_) {
      global_path_update_timer_.start();
      return;
    }
  }

  // Handle RTH mode and exploration mode separately
  if (has_goal_rth_) {
    // RTH mode: call planGoalPath and transition to PLAN_TRAJ_RTH
    int res = expl_manager_->planGoalPath(goal_rth_.head<3>(), goal_rth_(3));
    if (res == SUCCEED && state_ != WAIT_TRIGGER) {
      transitState(PLAN_TRAJ_RTH, "planGoalPath: succeed");
    } else if (res == FAIL) {
      // Keep current state, will retry on next timer callback
      ROS_WARN("RTH global path planning failed, will retry");
    }
    expl_manager_->frontier_manager_ptr_->viz_pocc();
    expl_manager_->frontier_manager_ptr_->visfrtcluster();
    global_path_update_timer_.start();
    return;
  }

  // Exploration mode: use TSP-based global planning
  cout << endl << endl;
  cout << "\033[1;33m------------- <" << cnt
       << "> Plan Global Path start---------------" << "\033[0m" << endl;
  planner_manager_->topo_graph_->log << "<" << cnt << ">" << endl;
  ros::Time t4 = ros::Time::now();
  // cout << "updateSkeleton time cost:" << (t3 - t2).toSec() * 1000 << "ms" <<
  // endl; if( (t3 - t1).toSec() * 1000 > 100){
  //   ROS_ERROR("time too long");
  //   exit(0);
  // }
  ROS_INFO("update topo skeleton cost: %fms, update odom vertex cost:%fms ",
           (t3 - t2).toSec() * 1000, (t4 - t3).toSec() * 1000);
  Eigen::Vector3d vel = fd_->odom_vel_.cast<double>();
  Eigen::Vector3d odom = fd_->odom_pos_.cast<double>();
  int res = expl_manager_->planGlobalPath(odom, vel);
  ros::Time t5 = ros::Time::now();

  cout << "\033[1;33m-------------Plan Global Path end-----------------"
       << "\033[0m" << endl
       << endl;

  planner_manager_->graph_visualizer_->vizBox(planner_manager_->topo_graph_);
  if(expl_manager_->ep_->view_graph_)
    planner_manager_->graph_visualizer_->vizGraph(planner_manager_->topo_graph_);
  std_msgs::Float32 time_cost;
  double time_cost_now = (t5 - t2).toSec() * 1000;
  time_cost.data = time_cost_now;
  time_cost_pub_.publish(time_cost);

  cout << "total time cost: " << time_cost_now << "ms" << endl;
  if (res == NO_FRONTIER && state_ != WAIT_TRIGGER) {
    transitState(FINISH, "planGlobalPath: no frontier");
  } else if (res == SUCCEED && state_ != WAIT_TRIGGER) {
    transitState(PLAN_TRAJ_EXP, "planGlobalPath: succeed");
  }

  expl_manager_->frontier_manager_ptr_->viz_pocc();
  expl_manager_->frontier_manager_ptr_->visfrtcluster();
  static ros::Time t_p = ros::Time::now();
  if ((ros::Time::now() - t_p).toSec() > 5.0) {
    expl_manager_->frontier_manager_ptr_->printMemoryCost();
    t_p = ros::Time::now();
  }
  global_path_update_timer_.start();
  cout << "viz&&print cost:" << (ros::Time::now() - t5).toSec() * 1000 << "ms"
       << endl;
}

void FastExplorationFSM::globalPathUpdateCallback(const ros::TimerEvent &e) {
  updateTopoAndGlobalPath();
}

bool FastExplorationFSM::goalServiceCallback(epic_planner::GoalService::Request& req,
                                             epic_planner::GoalService::Response& res) {
  goal_rth_ << req.x, req.y, req.z, req.yaw;
  has_goal_rth_ = true;

  ROS_INFO("\033[32m[RTH] Goal received: (%.2f, %.2f, %.2f), yaw: %.2f\033[0m",
           req.x, req.y, req.z, req.yaw);

  // Trigger state transition
  if (state_ == WAIT_TRIGGER || state_ == EXEC_TRAJ || state_ == PLAN_TRAJ_EXP) {
    transitState(PLAN_TRAJ_RTH, "Goal service called");
  }

  res.success = true;
  res.message = "Goal received, navigating to position";
  return true;
}

int FastExplorationFSM::callGoalPlanner() {
  ros::Time planning_start_time = ros::Time::now();

  // Check prerequisites
  if (planner_manager_->topo_graph_->odom_node_->neighbors_.empty())
    return START_FAIL;
  if (expl_manager_->ed_->global_tour_.size() < 2)
    return FAIL;

  Eigen::Vector3d goal_pos = goal_rth_.head<3>();
  double goal_yaw = goal_rth_(3);

  // Call exploration manager's goal planning function to generate global_tour_
  int res = expl_manager_->planGoalPath(goal_pos, goal_yaw);
  if (res != SUCCEED) {
    return res;
  }

  // Update next_goal_node_ from global_tour_
  expl_manager_->updateGoalNode();

  // Generate local trajectory using fast_searcher
  vector<Eigen::Vector3f> path_next_goal;
  res = planner_manager_->fast_searcher_->search(
      planner_manager_->topo_graph_->odom_node_,
      fd_->odom_vel_,
      expl_manager_->ed_->next_goal_node_,
      0.2, path_next_goal);

  if (res == ParallelBubbleAstar::NO_PATH) {
    ROS_ERROR("[RTH] No path to goal");
    return FAIL;
  } else if (res == ParallelBubbleAstar::START_FAIL) {
    ROS_ERROR("[RTH] Start point in occ");
    return START_FAIL;
  } else if (res == ParallelBubbleAstar::END_FAIL) {
    ROS_ERROR("[RTH] End point in occ");
    return FAIL;
  } else if (res == ParallelBubbleAstar::TIME_OUT) {
    ROS_ERROR("[RTH] Time out");
    return FAIL;
  }

  // Handle replanning from current trajectory
  auto info = &planner_manager_->local_data_;
  if (!fd_->static_state_) {
    double plan_finish_time_exp = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;
    if (plan_finish_time_exp > info->duration_) {
      plan_finish_time_exp = info->duration_;
    }
    Eigen::Vector3d start_exp = info->minco_traj_.getPos(plan_finish_time_exp);
    path_next_goal.insert(path_next_goal.begin(), start_exp.cast<float>());
  }

  // Resample path to avoid too long segments
  vector<Eigen::Vector3f> path_next_goal_tmp;
  path_next_goal_tmp.push_back(path_next_goal[0]);
  for (int i = 1; i < path_next_goal.size();) {
    Eigen::Vector3f end_pt = path_next_goal_tmp.back();
    if ((path_next_goal[i] - end_pt).norm() > 1.0) {
      Eigen::Vector3f dir = (path_next_goal[i] - end_pt).normalized();
      path_next_goal_tmp.push_back(end_pt + 1.0 * dir);
    } else if ((path_next_goal[i] - end_pt).norm() < 0.01) {
      i++;
    } else {
      path_next_goal_tmp.push_back(path_next_goal[i]);
      i++;
    }
  }

  expl_manager_->ed_->path_next_goal_.swap(path_next_goal_tmp);

  // Plan trajectory
  int result;
  if (planner_manager_->planExploreTraj(expl_manager_->ed_->path_next_goal_, fd_->static_state_)) {
    traj_utils::PolyTraj poly_traj_msg;
    planner_manager_->polyTraj2ROSMsg(poly_traj_msg, info->start_time_);
    fd_->newest_traj_ = poly_traj_msg;

    traj_utils::PolyTraj poly_yaw_traj_msg;
    planner_manager_->polyYawTraj2ROSMsg(poly_yaw_traj_msg, info->start_time_);
    fd_->newest_yaw_traj_ = poly_yaw_traj_msg;

    result = SUCCEED;
  } else {
    ROS_ERROR("[RTH] Failed to plan trajectory");
    result = FAIL;
  }

  // Block until minimum planning period has elapsed
  double elapsed = (ros::Time::now() - planning_start_time).toSec();
  if (elapsed < local_planning_min_period_) {
    double hold_time = local_planning_min_period_ - elapsed;
    ROS_INFO("\033[33m[Planning Hz Limit] Holding for %.3f ms (planning took %.3f ms, min period %.3f ms)\033[0m",
             hold_time * 1000.0, elapsed * 1000.0, local_planning_min_period_ * 1000.0);
    ros::Duration(hold_time).sleep();
  }
  return result;
}
