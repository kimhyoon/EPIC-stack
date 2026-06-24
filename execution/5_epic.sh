#!/usr/bin/env bash
# 5_epic.sh — EPIC 플래너 (real_flight.launch)
#   exploration_node + traj_server + px4_ctrl_bridge
#   ※ FAST-LIO(/Odometry,/cloud_registered) + mavros 가 먼저 떠 있어야 함
#   ※ OFFBOARD/arm 은 RC/QGC 로 수동 (auto_arm=false)
# 사용: ./5_epic.sh        (rviz 없음)
#       ./5_epic.sh --rviz (EPIC rviz 포함)
set -u
source /opt/ros/noetic/setup.bash
source /home/hmcl/stack1/devel/setup.bash

RVIZ=false
for a in "$@"; do { [ "$a" = "--rviz" ] || [ "$a" = "--epic_rviz" ]; } && RVIZ=true; done

echo "[epic] roslaunch epic_planner real_flight.launch rviz:=$RVIZ"
exec roslaunch epic_planner real_flight.launch rviz:="$RVIZ"
