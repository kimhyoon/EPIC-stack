#!/usr/bin/env bash
# stack1_host.sh — ⚠️ 호스트(컨테이너 바깥)에서 실행하는 8-pane 런처
#
# 컨테이너 안의 stack1.sh 를 docker exec 로 띄우고, 호스트 터미널이 그 tmux 에 붙는다.
#   - tmux 서버 + 모든 ROS 노드 = 컨테이너 안에서 실행
#   - 호스트엔 tmux 불필요 (컨테이너 tmux 화면을 -it TTY 로 보고 조작)
#
# 사용(호스트):
#   ./stack1_host.sh                 # rviz 없음
#   ./stack1_host.sh --epic_rviz     # EPIC rviz 켜기
#   CONTAINER=<이름> ./stack1_host.sh # 컨테이너 이름 다를 때 (docker ps 로 확인)
#
# 재접속(분리 후):  docker exec -it <컨테이너> tmux attach -t epic
# 전체 종료:        docker exec -it <컨테이너> tmux kill-session -t epic
set -u

CONTAINER="${CONTAINER:-epic_drone}"
EXEC="/home/hmcl/stack1/execution/stack1.sh"   # 컨테이너 내부 경로

# 컨테이너 떠 있는지 확인
if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "[host] 컨테이너 '$CONTAINER' 가 실행 중이 아님. docker ps 로 이름 확인 후"
  echo "       CONTAINER=<이름> $0 $* 로 실행하세요."
  exit 1
fi

exec docker exec -it \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e FCU_URL="${FCU_URL:-}" \
  "$CONTAINER" "$EXEC" "$@"
