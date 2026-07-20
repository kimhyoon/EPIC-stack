#!/usr/bin/env bash
# 1_livox.sh — Livox Mid360S 드라이버 (livox_ros_driver2)
#   -> /livox/lidar (CustomMsg), /livox/imu
# 사용: ./1_livox.sh         (rviz 없음)
#       ./1_livox.sh --rviz  (rviz 포함)
#       PUB_FREQ=30 ./1_livox.sh   (퍼블리시 레이트 변경; FAST-LIO odom 레이트가 이걸 따라감)
# 참고: publish_freq 를 올리면 FAST-LIO /Odometry 레이트↑ (EV 부드러움↑), 단 스캔당 점수↓·CPU↑
set -u
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
[ -f "${WORKSPACE_ROOT}/devel/setup.bash" ] || { echo "[livox] ${WORKSPACE_ROOT}/devel/setup.bash 없음 — 워크스페이스 먼저 빌드" >&2; exit 1; }
source /opt/ros/noetic/setup.bash
source "${WORKSPACE_ROOT}/devel/setup.bash"

LAUNCH=msg_MID360.launch
for a in "$@"; do [ "$a" = "--rviz" ] && LAUNCH=rviz_MID360.launch; done
PUB_FREQ="${PUB_FREQ:-20}"      # 기본 20Hz (드라이버 기본값 10 → FAST-LIO odom 20Hz로)

echo "[livox] roslaunch livox_ros_driver2 $LAUNCH publish_freq:=$PUB_FREQ"
exec roslaunch livox_ros_driver2 "$LAUNCH" publish_freq:="$PUB_FREQ"
