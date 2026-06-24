#!/usr/bin/env bash
# 2_fastlio.sh — FAST-LIO (Mid360)
#   /livox/lidar,/livox/imu 구독 -> /Odometry, /cloud_registered  (world=camera_init)
#   ※ 1_livox.sh 가 먼저 떠 있어야 함
# 사용: ./2_fastlio.sh        (rviz 없음)
#       ./2_fastlio.sh --rviz (rviz 포함)
set -u
source /opt/ros/noetic/setup.bash
source /home/hmcl/stack1/devel/setup.bash

RVIZ=false
for a in "$@"; do [ "$a" = "--rviz" ] && RVIZ=true; done

echo "[fastlio] roslaunch fast_lio mapping_mid360.launch rviz:=$RVIZ"
exec roslaunch fast_lio mapping_mid360.launch rviz:="$RVIZ"
