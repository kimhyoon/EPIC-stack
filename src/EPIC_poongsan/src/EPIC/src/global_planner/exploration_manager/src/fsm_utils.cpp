#include <epic_planner/expl_data.h>
#include <epic_planner/fast_exploration_fsm.h>
#include <frontier_manager/global_log.h>
#include <cmath>

void FastExplorationFSM::pubState() {

  std_msgs::Empty heartbeat_msg;
  heartbeat_pub_.publish(heartbeat_msg);
  std_msgs::Bool msg;
  msg.data = fd_->static_state_;
  static_pub_.publish(msg);
  Marker state_marker;
  state_marker.type = Marker::TEXT_VIEW_FACING;
  state_marker.pose.position.x = fd_->odom_pos_.x();
  state_marker.pose.position.y = fd_->odom_pos_.y();
  state_marker.pose.position.z = fd_->odom_pos_.z();
  state_marker.pose.orientation.w = 1.0;
  state_marker.scale.x = state_marker.scale.y = state_marker.scale.z = 0.5;
  state_marker.action = Marker::ADD;
  state_marker.color.r = 1.0;
  state_marker.color.a = 1.0;
  // 표시만 오버라이드: 회피 MUX 가 조종 중이면 FSM 상태 대신 AVOIDANCE 를 그린다.
  // FSM 상태 자체는 안 바뀐다 (판정식은 FSMCallback 상단의 avoiding 과 동일).
  const bool avoiding_now =
      avoidance_enabled_ && have_avoid_flag_ && (avoid_flag_ == 1) &&
      ((ros::Time::now() - last_avoid_flag_stamp_).toSec() < avoid_flag_timeout_);
  state_marker.text = avoiding_now ? std::string("AVOIDANCE")
                                   : fd_->state_str_[int(state_)];
  state_marker.header.frame_id = "odom";
  state_marker.header.stamp = ros::Time::now();

  state_pub_.publish(state_marker);
}

int FastExplorationFSM::callExplorationPlanner() {
  ros::Time planning_start_time = ros::Time::now();

  // if (planner_manager_->lidar_map_interface_->getDisToOcc(fd_->odom_pos_) < planner_manager_->gcopter_config_->dilateRadiusHard)
  //   return START_FAIL;
  if (planner_manager_->topo_graph_->odom_node_->neighbors_.empty())
    return START_FAIL;
  if (expl_manager_->ed_->global_tour_.size() < 2)
    return NO_FRONTIER;
  if (!expl_manager_->ed_->next_goal_node_) {
    local_reason_ = "global tour has no virtual goal node";
    EPIC_LOG_ERROR("local.cycle", "%s", local_reason_.c_str());
    return FAIL;
  }

  // debug
  if (planner_manager_->lidar_map_interface_->getDisToOcc(expl_manager_->ed_->next_goal_node_->center_) <=
      planner_manager_->parallel_path_finder_->safe_distance_) {
    local_reason_ = "next goal too close to occupancy -> re-run global";
    EPIC_LOG_WARN_THROTTLE(2.0, "local.cycle", "%s", local_reason_.c_str());
    updateTopoAndGlobalPath();
    return FAIL;
  }
  vector<Eigen::Vector3f> path_next_goal;

  int res = planner_manager_->fast_searcher_->search(planner_manager_->topo_graph_->odom_node_, fd_->odom_vel_, expl_manager_->ed_->next_goal_node_,
                                                     0.2, path_next_goal);
  if (res == ParallelBubbleAstar::NO_PATH) {
    local_reason_ = "fast-searcher: no path odom->goal";
    EPIC_LOG_WARN_THROTTLE(2.0, "local.cycle", "%s", local_reason_.c_str());
    return FAIL;

  } else if (res == ParallelBubbleAstar::START_FAIL) {
    local_reason_ = "fast-searcher: start(odom) in occupancy";
    EPIC_LOG_WARN_THROTTLE(2.0, "local.cycle", "%s", local_reason_.c_str());
    return START_FAIL;
  } else if (res == ParallelBubbleAstar::END_FAIL) {
    local_reason_ = "fast-searcher: goal in occupancy";
    EPIC_LOG_WARN_THROTTLE(2.0, "local.cycle", "%s", local_reason_.c_str());
    return FAIL;
  } else if (res == ParallelBubbleAstar::TIME_OUT) {
    local_reason_ = "fast-searcher: timeout";
    EPIC_LOG_WARN_THROTTLE(2.0, "local.cycle", "%s", local_reason_.c_str());
    return FAIL;
  }

  auto info = &planner_manager_->local_data_;

  // planExploreTraj now anchors PVA and P0 to one atomic live-odometry
  // snapshot. Do not prepend a future sample from the previous command here:
  // that made the guide path start and optimizer start differ by the vehicle's
  // tracking lead, producing fake fallback progress and trajectory steps.
  vector<Eigen::Vector3f> path_next_goal_tmp;
  path_next_goal_tmp.push_back(path_next_goal[0]);

  for (int i = 1; i < path_next_goal.size();) {
    Eigen::Vector3f end_pt = path_next_goal_tmp.back();
    const Eigen::Vector3f segment = path_next_goal[i] - end_pt;
    const double segment_norm = segment.norm();
    if (segment.allFinite() && segment_norm > 1.0) {
      Eigen::Vector3f dir = segment / segment_norm;
      path_next_goal_tmp.push_back(end_pt + 1.0 * dir);
    } else if (!std::isfinite(segment_norm) || segment_norm < 0.01) {
      i++;
    } else {
      path_next_goal_tmp.push_back(path_next_goal[i]);
      i++;
    }
  }
  expl_manager_->ed_->path_next_goal_.swap(path_next_goal_tmp);
  int result;
  if (planner_manager_->planExploreTraj(expl_manager_->ed_->path_next_goal_, fd_->static_state_)) {
    traj_utils::PolyTraj poly_traj_msg;
    planner_manager_->polyTraj2ROSMsg(poly_traj_msg, info->start_time_);
    fd_->newest_traj_ = poly_traj_msg;
    traj_utils::PolyTraj poly_yaw_traj_msg;
    planner_manager_->polyYawTraj2ROSMsg(poly_yaw_traj_msg, info->start_time_);
    fd_->newest_yaw_traj_ = poly_yaw_traj_msg;
    local_reason_ = planner_manager_->last_plan_was_escape_ ? "escape traj (flyToSafeRegion)" : "";
    result = SUCCEED;
  } else {
    local_reason_ = planner_manager_->last_plan_fail_reason_.empty()
                        ? "traj optimization failed"
                        : planner_manager_->last_plan_fail_reason_;
    result = FAIL;
  }

  // Block until minimum planning period has elapsed
  double elapsed = (ros::Time::now() - planning_start_time).toSec();
  if (elapsed < local_planning_min_period_) {
    double hold_time = local_planning_min_period_ - elapsed;
    ROS_DEBUG("[Planning Hz Limit] Holding for %.3f ms (planning took %.3f ms, min period %.3f ms)",
              hold_time * 1000.0, elapsed * 1000.0, local_planning_min_period_ * 1000.0);
    ros::Duration(hold_time).sleep();
  }
  return result;
}

// 미션 시작 공용 진입점. 트리거 소스와 무관하게 동일 동작:
//  - rviz 2D Nav Goal (-> waypoint_generator -> /waypoint_generator/waypoints)
//  - rostopic pub /waypoint_generator/waypoints ... (직접 발행)
//  - rosservice call /srv_start (rviz 없는 환경용)
bool FastExplorationFSM::startMission(const std::string &source) {
  if (state_ != WAIT_TRIGGER)
    return false;
  early_finish_count_ = 0;
  clearRthFailures();
  early_finish_force_requested_ = false;
  clearEarlyFinishPath("IDLE");
  traveled_distance_ = 0.0;
  traveled_valid_ = false;
  max_displacement_ = 0.0;
  expl_origin_ = fd_->odom_pos_;  // 임시 기준점 — startExplorationFromHover 가 재앵커
  frontiers_ever_seen_ = false;
  explore_start_time_ = ros::Time(0);
  finish_hover_start_ = ros::Time(0);
  fd_->trigger_ = true;
  total_time_ = ros::Time::now().toSec();
  // 미션 t0 재설정(+상대시간이 트리거 기준이 됨) + 파라미터 스냅샷 재발행
  // (레코더가 이벤트 스트림을 받는 시점 이후에 남도록)
  elog_.markMissionStart();
  char tp[96];
  snprintf(tp, sizeof(tp), "trigger received | pos=(%.2f, %.2f, %.2f)",
           fd_->odom_pos_.x(), fd_->odom_pos_.y(), fd_->odom_pos_.z());
  elog_.log("EVENT", "mission start (" + source + ")", tp, 0.0,
            EventLogger::L_INFO, true);
  logParamsEvents(true);

  if (fp_->takeoff_height_ > 0.0) {
    // Climb to the configured altitude and hover, then auto-start exploration once
    // odom confirms we're stable near it. Hold current x,y and heading during climb.
    takeoff_anchor_ = Eigen::Vector3d(fd_->odom_pos_.x(), fd_->odom_pos_.y(),
                                      fp_->takeoff_height_);
    takeoff_yaw_ = fd_->odom_yaw_;
    // [offboard landing] 이륙 "지면"의 z 를 따로 남긴다. takeoff_anchor_.z 는 목표
    // 호버 고도(설정 절대값)라서 지면이 아니다. 착륙 시 AGL = odom_z - land_ground_z_
    // 로 쓰므로, 이 값이 없으면 지면이 어디인지 알 방법이 없다.
    land_ground_z_ = fd_->odom_pos_.z();
    land_ground_z_valid_ = true;
    hover_enter_time_ = ros::Time::now();
    hover_stable_since_ = ros::Time(0);
    transitState(TAKEOFF_HOVER, source + ": takeoff & hover");
  } else {
    // takeoff_height disabled -> original behaviour (start exploring immediately).
    transitState(PLAN_TRAJ_EXP, source);
  }
  return true;
}

void FastExplorationFSM::triggerCallback(const nav_msgs::PathConstPtr &msg) {
  if (msg->poses.empty() || msg->poses[0].pose.position.z < -0.1)
    return;
  startMission("waypoints trigger");
}

void FastExplorationFSM::avoidFlagCallback(const std_msgs::Int16ConstPtr &msg) {
  // Reactive-avoidance flag from the local_avoidance node (1 = obstacle close).
  // Just latch it + stamp; the FSMCallback acts on it synchronously with planning.
  avoid_flag_ = msg->data;
  last_avoid_flag_stamp_ = ros::Time::now();
  have_avoid_flag_ = true;
}

void FastExplorationFSM::CloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr &msg, const nav_msgs::Odometry::ConstPtr &odom_) {
  ros::Time t1 = ros::Time::now();
  if (!planner_manager_->lidar_map_interface_->updateCloudMapOdometry(msg,
                                                                      odom_))
    return;
  double collision_time = planner_manager_->local_data_.duration_;
  const bool rotate_hold = planner_manager_->isRotateInPlaceHoldActive();
  // A rotate-in-place recovery samples the same position for its whole
  // duration. That point may already be inside the inflated obstacle margin,
  // which is exactly why recovery was requested; rechecking the identical
  // point every cloud frame aborts the yaw motion after 30--100 ms. Suppress
  // only that self-collision for the explicitly marked trajectory duration.
  // CAUTION has a different collision invariant from an ordinary trajectory:
  // its exact start can already be inside the soft/hard margin.  Its owning
  // FSM therefore checks the certified raw-to-soft escape below instead of
  // feeding that unavoidable t=0 condition to the ordinary checker.
  bool safe = rotate_hold || state_ == CAUTION || state_ == MAP_REBUILD ||
              planner_manager_->checkTrajCollision(collision_time);
  // CAUTION 은 스스로 풀릴 때까지 건드리지 않는다. CAUTION 은 flyToSafeRegion 으로
  // 탈출 궤적을 만들고, 그게 실제로 기체를 안전거리 밖으로 빼놓았는지를
  // getDisToOcc(odom) > dilateRadiusSoft 로 직접 확인한 뒤에야 나간다
  // (fast_exploration_fsm.cpp CAUTION case). 그런데 탈출 궤적은 "지금 장애물에
  // 붙어 있으니 빠져나가는" 궤적이라 일반 충돌검사를 통과하지 못한다 — 여기서
  // 그걸 근거로 상태를 되돌리면 CAUTION 이 탈출을 완주하지 못하고 매 클라우드
  // 프레임마다 PLAN_TRAJ_EXP 로 끌려나간다(실측: 0804 09-06-08 bag, CAUTION 진입
  // 2회에 강제 이탈 9회, ~0.24s 주기 flapping). stopTraj() 도 같은 이유로 막는다 —
  // 방금 발행한 탈출 궤적을 잘라버리기 때문이다.
  // 대신 CAUTION 전용 검사는 현재점보다 clearance 를 악화시키지 않는 egress와,
  // soft margin 도달 후 유지, raw/safe FIRI, observed boundary, ceiling 을 갱신된
  // 맵마다 검사한다. 실패해도 PLANNING 으로 넘기지 않고 CAUTION 이 궤적을
  // 중단하고 현재 odom 에서 다시 만든다.
  if (state_ == CAUTION && caution_phase_ == CAUTION_EXEC_ESCAPE) {
    const Eigen::Vector3d actual_position(odom_->pose.pose.position.x,
                                          odom_->pose.pose.position.y,
                                          odom_->pose.pose.position.z);
    double escape_violation_time = 0.0;
    std::string escape_violation_reason;
    if (!planner_manager_->checkCautionEscapeSafety(
            actual_position, escape_violation_time,
            escape_violation_reason)) {
      EPIC_LOG_ERROR_THROTTLE(
          1.0, "local.collision",
          "CAUTION live escape safety violation at t=%.3f: %s; "
          "stopping and rebuilding inside CAUTION",
          escape_violation_time, escape_violation_reason.c_str());
      stopTraj();
      caution_phase_ = CAUTION_IDLE;
      caution_escape_traj_id_ = 0;
      ++caution_escape_fail_count_;
    }
  }
  // 맵 갱신/odom 갱신/frontier 갱신 등 이 콜백의 나머지 일은 CAUTION 중에도 그대로 돈다.
  //
  // LAND/LANDED 도 같은 이유로 제외한다. 착륙 중에는 local_data_ 에 착륙 직전의
  // 마지막 궤적이 그대로 남아 있고, 기체가 지면에 가까워질수록 그 궤적은 당연히
  // 충돌검사를 통과하지 못한다. 그걸 근거로 PLAN_TRAJ_EXP 로 튀면 AUTO.LAND 재요청
  // 루프와 착지(disarm) 감지가 통째로 중단되어 착륙이 취소된다.
  // PILOT_OVERRIDE 는 terminal latch 이므로 cloud collision 결과도 상태를 다시
  // PLAN_TRAJ_* 로 바꾸지 못한다.
  if (!safe && state_ != CAUTION && state_ != MAP_REBUILD &&
      state_ != LAND && state_ != LANDED && state_ != PILOT_OVERRIDE) {
    const bool rotate_needs_escape =
        planner_manager_->local_data_.rotate_in_place_ &&
        planner_manager_->lidar_map_interface_->getDisToOcc(
            Eigen::Vector3d(odom_->pose.pose.position.x,
                            odom_->pose.pose.position.y,
                            odom_->pose.pose.position.z)) <=
            planner_manager_->gcopter_config_->dilateRadiusSoft;
    EXPL_STATE next_state = rotate_needs_escape
                                ? CAUTION
                                : (has_goal_rth_ ? PLAN_TRAJ_RTH
                                                 : PLAN_TRAJ_EXP);
    const double trajectory_elapsed = std::max(
        0.0, (ros::Time::now() - planner_manager_->local_data_.start_time_)
                 .toSec());
    const double collision_eta =
        std::max(0.0, collision_time - trajectory_elapsed);
    if (has_goal_rth_ && !rotate_needs_escape &&
        noteRthFailure("cloud trajectory collision"))
      return;
    transitState(next_state,
                 "safetyCallback: not safe, traj_time:" +
                     to_string(collision_time) +
                     " eta:" + to_string(collision_eta),
                 true);
    if (!rotate_needs_escape &&
        collision_eta < fp_->replan_time_ + 0.2)
      stopTraj();
  }
  ros::Time t2 = ros::Time::now();
  ros::Time t3 = ros::Time::now();

  if (planner_manager_->lidar_map_interface_->ld_->lidar_cloud_.points.empty())
    return;
  auto& ld = planner_manager_->lidar_map_interface_->ld_;
  fd_->odom_pos_ = ld->lidar_pose_;
  fd_->odom_vel_ = ld->lidar_vel_;

  // 이동량 갱신. EARLY_FINISH 판정 근거는 max_displacement_ (탐사 시작점 기준
  // 최대 직선 이탈거리, 단조증가) — 경로 적분(traveled_distance_)은 근거리 셔플
  // 만으로 임계값을 소진해 구제를 죽이는 문제가 있어 진단용으로만 남긴다.
  // 한 프레임(20Hz) 이동량 상한 1m = 20m/s 로, 정상 비행에서는 나올 수 없는 값이다.
  // 이걸 넘으면 FAST-LIO/LIO-SAM 포즈 점프이므로 두 지표 모두 갱신하지 않는다.
  // (점프가 지속되면 그 이후 max 가 오염되는 건 못 막지만, 그 상태면 지도/계획
  // 전체가 이미 깨진 상황이라 여기서 더 방어하지 않는다.)
  if (traveled_valid_) {
    double step = (fd_->odom_pos_ - last_traveled_pos_).norm();
    if (std::isfinite(step) && step < 1.0) {
      traveled_distance_ += step;
      double disp = (fd_->odom_pos_ - expl_origin_).norm();
      if (std::isfinite(disp) && disp > max_displacement_)
        max_displacement_ = disp;
    }
  }
  last_traveled_pos_ = fd_->odom_pos_;
  traveled_valid_ = true;

  const double odom_yaw = tf::getYaw(odom_->pose.pose.orientation);
  if (!std::isfinite(odom_yaw)) {
    EPIC_LOG_WARN_THROTTLE(1.0, "sensor.lidar",
                           "ignoring non-finite odometry yaw");
    return;
  }
  fd_->odom_yaw_ = static_cast<float>(odom_yaw);
  planner_manager_->local_data_.curr_pos_ = fd_->odom_pos_.cast<double>();
  planner_manager_->local_data_.curr_vel_ = fd_->odom_vel_.cast<double>();
  planner_manager_->topo_graph_->odom_node_->center_ = fd_->odom_pos_;
  fd_->have_odom_ = true;
  vector<ClusterInfo::Ptr> new_clusters;
  vector<int> cluster_removed;
  expl_manager_->frontier_manager_ptr_->updateFrontierClusters(new_clusters, cluster_removed);
  for (auto &cls : new_clusters) {
    cls->odom_id_ = planner_manager_->topo_graph_->history_odom_nodes_.size() - 1;
  }
  ros::Time t4 = ros::Time::now();

  if (verbose_console_)
    ROS_INFO_STREAM_THROTTLE(1.0, "cloud odom callback cost: " << "ikd-tree insert:" << (t2 - t1).toSec() * 1000 << "ms  "
                                                               << "update frontier clusters: " << (t4 - t3).toSec() * 1000 << "ms  "
                                                               << "total: " << (t4 - t1).toSec() * 1000 << "ms" << endl);
}

void FastExplorationFSM::transitState(EXPL_STATE new_state, string pos_call, bool red) {
  // Once control has left OFFBOARD the planner must not silently resume from a
  // timer, cloud callback, or service. A new mission requires a planner restart.
  if (state_ == PILOT_OVERRIDE && new_state != PILOT_OVERRIDE) {
    EPIC_LOG_WARN_THROTTLE(2.0, "execution.fsm",
                           "ignoring state transition out of PILOT_OVERRIDE to %d (%s)",
                           int(new_state), pos_call.c_str());
    return;
  }
  int pre_s = int(state_);
  const std::string previous = fd_->state_str_[pre_s];
  const std::string next = fd_->state_str_[int(new_state)];
  if (new_state == CAUTION && state_ != CAUTION) {
    caution_phase_ = CAUTION_IDLE;
    caution_escape_traj_id_ = 0;
    caution_enter_time_ = ros::Time::now();
    caution_last_attempt_time_ = ros::Time(0);
    caution_escape_fail_count_ = 0;
  }
  // [offboard landing] LAND 에 들어오는 순간의 포즈를 하강 앵커로 고정한다.
  // xy 는 여기서 잡은 값을 착지까지 바꾸지 않는다 (드리프트를 따라가지 않도록).
  if (new_state == LAND && state_ != LAND) {
    land_phase_ = LAND_PHASE_DESCEND;
    land_enter_time_ = ros::Time::now();
    land_last_tick_ = land_enter_time_;
    land_touch_since_ = ros::Time(0);
    land_disarm_since_ = ros::Time(0);
    land_last_req_ = ros::Time(0);
    land_bias_logged_ = false;
    land_touchdown_confirmed_ = false;
    // xy 는 "LAND 를 시작한 자리"가 아니라 기억해 둔 이륙 좌표를 쓴다. RTH 는
    // rth_land_xy_tol(0.2m) 안에 들어오면 LAND 로 넘어오므로 둘은 최대 그만큼
    // 차이나는데, 하강 내내 붙잡을 기준은 원점이어야 한다. (takeoff 가 비활성이면
    // 이륙 좌표가 없으므로 현재 위치로 폴백.)
    land_xy_anchor_ = fd_->odom_pos_.cast<double>();
    if (fp_->takeoff_height_ > 0.0) {
      land_xy_anchor_.x() = takeoff_anchor_.x();
      land_xy_anchor_.y() = takeoff_anchor_.y();
    }
    land_z_ramp_ = fd_->odom_pos_.z();
    land_yaw_ = fd_->odom_yaw_;
  }
  state_ = new_state;
  // 이벤트 로거가 상태전이를 기록한다. PLAN<->EXEC 리플랜 플래핑 같은 A<->B 교대
  // 패턴은 로거의 사이클 억제가 걸러 "cycling xN" heartbeat 만 남긴다.
  elog_.setState(fd_->state_str_[int(new_state)]);
  elog_.log("STATE",
            previous + " -> " + next + " [" + pos_call + "]",
            "", 0.0, red ? EventLogger::L_WARN : EventLogger::L_INFO);
  if (red)
    EPIC_LOG_WARN_THROTTLE(1.0, "execution.fsm",
                           "state=%s->%s cause=%s", previous.c_str(),
                           next.c_str(), pos_call.c_str());
  else
    EPIC_LOG_DEBUG(1, 1, "execution.fsm", "state=%s->%s cause=%s",
                   previous.c_str(), next.c_str(), pos_call.c_str());
}

void FastExplorationFSM::stopTraj() {
  replan_pub_.publish(std_msgs::Empty());
  ros::Time time_now = ros::Time::now();
  ros::Time start_time = planner_manager_->local_data_.start_time_;
  double curr_dur = planner_manager_->local_data_.duration_;
  planner_manager_->local_data_.duration_ = min(curr_dur, (time_now - start_time).toSec() + fp_->replan_time_);
  if (planner_manager_->local_data_.duration_ <= (time_now - start_time).toSec())
    fd_->static_state_ = true;
}
