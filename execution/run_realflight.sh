#!/usr/bin/env bash
#
# run_realflight.sh — EPIC 실비행 풀스택 일괄 실행
#
#   0) roscore (없으면 시작)
#   1) livox_ros_driver2  : Mid360S -> /livox/lidar, /livox/imu  (rviz 없음)
#   2) FAST-LIO (mid360)  : -> /Odometry, /cloud_registered  (world=camera_init, rviz 끔)
#   3) mavros (px4)       : FCU 연결 -> /mavros/*
#   4) static TF x2       : odom->camera_init, body->base_link  (ENU identity, 트리 연결)
#   5) throttle           : /Odometry -> /mavros/odometry/out @20Hz  (PX4 EKF2 external vision)
#   6) EPIC               : real_flight.launch (planner + traj_server + px4_ctrl_bridge)
#
# 모든 노드를 순서대로 백그라운드로 띄우고, Ctrl+C 한 번이면 전부 종료됩니다.
# 로그: $LOG_DIR/<name>.log
#
# 사용법:
#   ./run_realflight.sh                      # rviz 전부 끔 (헤드리스)
#   ./run_realflight.sh --epic_rviz          # EPIC rviz 만 켜기
#   FCU_URL=/dev/ttyUSB0:57600 ./run_realflight.sh
#
# 주의: OFFBOARD 진입/arm 은 사람이 RC/QGC로 (real_flight.launch 의 auto_arm=false).
#       첫 비행은 프로펠러 떼거나 SITL 권장.

set -u

# ---- 옵션 파싱 ------------------------------------------------------------
EPIC_RVIZ=false                     # 기본: EPIC rviz 끔
for arg in "$@"; do
  case "$arg" in
    --epic_rviz) EPIC_RVIZ=true ;;
    *) echo "[realflight] 알 수 없는 옵션: $arg" ;;
  esac
done

# ---- 설정 -----------------------------------------------------------------
ROS_SETUP="/opt/ros/noetic/setup.bash"
STACK_WS="/home/hmcl/stack1/devel/setup.bash"
FCU_URL="${FCU_URL:-/dev/ttyACM0:921600}"
LOG_DIR="/home/hmcl/stack1/logs"
mkdir -p "$LOG_DIR"

# 메인 셸에서 rostopic 등을 쓰기 위해 미리 source (백그라운드 서브셸도 상속)
source "$ROS_SETUP"
source "$STACK_WS"

PIDS=()

# ---- 종료 처리 ------------------------------------------------------------
cleanup() {
  echo
  echo "[realflight] 종료 중 ..."
  # 역순으로 정리
  for ((i=${#PIDS[@]}-1; i>=0; i--)); do
    kill -INT "${PIDS[$i]}" 2>/dev/null
  done
  wait 2>/dev/null
  echo "[realflight] 완료."
}
trap cleanup INT TERM EXIT

# bg <name> <command...>  : ROS 환경에서 백그라운드 실행 + 로깅
bg() {
  local name="$1"; shift
  echo "[realflight] $name 실행 ..."
  ( source "$ROS_SETUP"; source "$STACK_WS"; exec "$@" ) > "$LOG_DIR/$name.log" 2>&1 &
  PIDS+=("$!")
}

# wait_topic <topic> [timeout_s] : 토픽이 advertise 될 때까지 대기
wait_topic() {
  local topic="$1" to="${2:-30}" i=0
  echo "[realflight]   -> $topic 대기 (최대 ${to}s)..."
  until rostopic list 2>/dev/null | grep -qx "$topic"; do
    sleep 1; i=$((i+1))
    if [ "$i" -ge "$to" ]; then
      echo "[realflight]   ⚠️ $topic 안 뜸 — 그래도 계속 진행"
      return 1
    fi
  done
  echo "[realflight]   -> $topic OK"
}

# ---- 0) roscore -----------------------------------------------------------
if ! rostopic list >/dev/null 2>&1; then
  bg roscore roscore
  echo "[realflight]   -> roscore 기동 대기 (3s)"
  sleep 3
else
  echo "[realflight] roscore 이미 실행 중 — 기존 마스터 사용"
fi

# ---- 1) livox 드라이버 (rviz 없음: msg_MID360) -----------------------------
bg livox roslaunch livox_ros_driver2 msg_MID360.launch
wait_topic /livox/lidar 20

# ---- 2) FAST-LIO (rviz 끔) ------------------------------------------------
bg fastlio roslaunch fast_lio mapping_mid360.launch rviz:=false
wait_topic /Odometry 30          # FAST-LIO 수렴까지 시간 좀 걸릴 수 있음

# ---- 3) mavros (FCU) ------------------------------------------------------
bg mavros roslaunch mavros px4.launch fcu_url:="$FCU_URL"
echo "[realflight]   -> FCU 연결 대기 (5s)"
sleep 5

# ---- 4) static TF (ENU identity, FAST-LIO 트리 <-> mavros 트리 연결) -------
bg tf_odom_caminit  rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 odom camera_init
bg tf_body_baselink rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 body base_link
sleep 1

# ---- 5) /Odometry -> /mavros/odometry/out (PX4 EKF2 external vision) -------
bg odom_throttle rosrun topic_tools throttle messages /Odometry 20.0 /mavros/odometry/out
sleep 2

# ---- 6) EPIC --------------------------------------------------------------
bg epic roslaunch epic_planner real_flight.launch rviz:="$EPIC_RVIZ"

echo
echo "[realflight] 모두 실행됨. 로그: $LOG_DIR/*.log"
echo "[realflight] 점검:"
echo "    rostopic hz /Odometry /livox/lidar"
echo "    rostopic echo /mavros/state | grep -E 'connected|mode|armed'"
echo "    rostopic echo -n1 /mavros/local_position/odom   # FAST-LIO 따라 움직이면 EKF 융합 OK"
echo "[realflight] 비행: RC/QGC로 arm -> OFFBOARD, 그다음 rviz에서 goal 찍어 EPIC 트리거"
echo "[realflight] 종료: Ctrl+C"
wait
