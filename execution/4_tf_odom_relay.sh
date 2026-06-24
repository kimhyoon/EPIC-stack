#!/usr/bin/env bash
# 4_tf_odom_relay.sh — FAST-LIO -> PX4 EKF 연결용 TF + odom 릴레이
#   - static TF: odom->camera_init, body->base_link  (ENU identity, 트리 연결)
#   - relay    : /Odometry -> /mavros/odometry/out  (풀 레이트, PX4 EKF2 external vision)
#   ※ 2_fastlio.sh(/Odometry) + 3_mavros.sh 가 먼저 떠 있어야 함
#   세 프로세스를 함께 띄우고 Ctrl+C 한 번에 전부 종료.
set -u
source /opt/ros/noetic/setup.bash
source /home/hmcl/stack1/devel/setup.bash

PIDS=()
cleanup() {
  echo; echo "[tf_relay] 종료 중 ..."
  for p in "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null; done
  wait 2>/dev/null
}
trap cleanup INT TERM EXIT

echo "[tf_relay] static TF: odom->camera_init"
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 odom camera_init & PIDS+=("$!")

echo "[tf_relay] static TF: body->base_link"
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 body base_link & PIDS+=("$!")

# EPIC frontier_viszer 가 occ/pocc/frt 복셀을 frame_id="world" 로 발행(하드코딩)하는데
# 시스템 나머지는 camera_init 프레임 → world 를 camera_init 자식(identity)으로 붙여 트리에 연결.
# (이거 없으면 rviz 에 voxel 이 'world 프레임 없음' 으로 안 뜸)
echo "[tf_relay] static TF: camera_init->world (EPIC voxel viz 용)"
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 camera_init world & PIDS+=("$!")

echo "[tf_relay] relay /Odometry -> /mavros/odometry/out (풀 레이트)"
rosrun topic_tools relay /Odometry /mavros/odometry/out & PIDS+=("$!")

echo "[tf_relay] 실행 중. Ctrl+C 로 전체 종료."
wait
