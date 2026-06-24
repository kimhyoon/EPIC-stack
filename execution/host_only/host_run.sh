#!/usr/bin/env bash
# host_run.sh  — ⚠️ 호스트(컨테이너 바깥)에서 실행하는 래퍼
#
# 컨테이너 안의 execution 스크립트를 docker exec 로 실행한다.
# (컨테이너 안에 ROS/워크스페이스가 있으므로 직접 실행 대신 이걸 통해 실행)
#
# 사용(호스트):
#   ./host_run.sh 1_livox.sh
#   ./host_run.sh 2_fastlio.sh
#   ./host_run.sh 3_mavros.sh
#   ./host_run.sh 4_tf_odom_relay.sh
#   ./host_run.sh 5_epic.sh --rviz
#   CONTAINER=<다른이름> ./host_run.sh 3_mavros.sh        # 컨테이너 이름 다를 때
#   FCU_URL=/dev/ttyUSB0:57600 CONTAINER=epic_drone ./host_run.sh 3_mavros.sh
#
# 컨테이너 이름 확인:  docker ps   (NAMES 열)
set -u

CONTAINER="${CONTAINER:-epic_drone}"      # 기본 컨테이너 이름 (docker ps 로 확인)
EXEC_DIR="/home/hmcl/stack1/execution"    # 컨테이너 내부 경로

if [ $# -lt 1 ]; then
  echo "usage: $0 <script.sh> [args...]"
  echo "  예: $0 5_epic.sh --rviz"
  exit 1
fi
script="$1"; shift

# -it: 터미널 + Ctrl+C 전달,  -e: GUI(rviz) 위해 DISPLAY/FCU_URL 전달
exec docker exec -it \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e FCU_URL="${FCU_URL:-}" \
  "$CONTAINER" "$EXEC_DIR/$script" "$@"
