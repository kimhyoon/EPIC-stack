#!/usr/bin/env bash
# stack_all_host.sh — ⚠️ 호스트(컨테이너 바깥)에서 실행하는 8-pane 런처
#
# 컨테이너 안의 stack_all.sh 를 docker exec 로 띄우고, 호스트 터미널이 그 tmux 에 붙는다.
#   - tmux 서버 + 모든 ROS 노드 = 컨테이너 안에서 실행
#   - 호스트엔 tmux 불필요 (컨테이너 tmux 화면을 -it TTY 로 보고 조작)
#
# 사용(호스트):
#   ./stack_all_host.sh                 # rviz 없음
#   ./stack_all_host.sh --epic_rviz     # EPIC rviz 켜기
#   CONTAINER=<이름> ./stack_all_host.sh # 컨테이너 이름 다를 때 (docker ps 로 확인)
#
# 재접속(분리 후):  docker exec -it <컨테이너> tmux attach -t epic
# 전체 종료:        docker exec -it <컨테이너> tmux kill-session -t epic
set -u

CONTAINER="${CONTAINER:-epic_drone}"
# 컨테이너 내부 stack_all.sh 경로 — 기본값은 이 스크립트 위치에서 유추
# (워크스페이스가 컨테이너에 같은 경로로 마운트됐다는 가정).
# 다르면: EXEC=/컨테이너/내부/execution/stack_all.sh ./stack_all_host.sh ...
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
EXEC="${EXEC:-$(cd -- "${SCRIPT_DIR}/.." && pwd)/stack_all.sh}"

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
