#!/usr/bin/env bash
# 3_mavros.sh — MAVROS (PX4 FCU 브리지, ENU<->NED 변환)
#   -> /mavros/*  (state, local_position/odom, setpoint_raw/local, odometry/out ...)
# 사용: ./3_mavros.sh
#       FCU_URL=/dev/ttyUSB0:57600 ./3_mavros.sh
set -u
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
[ -f "${WORKSPACE_ROOT}/devel/setup.bash" ] || { echo "[mavros] ${WORKSPACE_ROOT}/devel/setup.bash 없음 — 워크스페이스 먼저 빌드" >&2; exit 1; }
source /opt/ros/noetic/setup.bash
source "${WORKSPACE_ROOT}/devel/setup.bash"

FCU_URL="${FCU_URL:-/dev/ttyUSB0:921600}"

echo "[mavros] roslaunch mavros px4.launch fcu_url:=$FCU_URL"
exec roslaunch mavros px4.launch fcu_url:="$FCU_URL"
