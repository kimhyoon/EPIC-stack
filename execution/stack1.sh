#!/usr/bin/env bash
# stack1.sh — tmux 한 세션을 8칸으로 쪼개 전체 스택을 각 pane에서 실행
#   (컨테이너 안에서 실행. 호스트에서 돌리려면: host_only/stack1_host.sh)
#
#   pane 0 roscore | 1 livox | 2 fastlio | 3 mavros
#        4 tf_relay| 5 epic   | 6 monitor| 7 free-shell
#
# 기본 동작:
#   각 pane에 명령이 "타이핑만" 되어 있고 자동 실행 안 됨 → 사용자가 Enter 쳐서 실행.
#   (환경 source 는 미리 자동으로 해둠. 권장 Enter 순서: 0→1→2→3→4→5)
#
# 옵션:
#   --autorun       모든 pane 자동 실행 (의존순서 위해 sleep 스태거 적용)
#   --epic_rviz     EPIC rviz 켜기
#   --fastlio_rviz  FAST-LIO rviz 켜기
#   SESSION=foo     세션 이름 변경
#
# 조작:  Ctrl-b 화살표(pane 이동, 마우스도 켜짐) / Ctrl-b z (pane 확대/복귀)
# 종료:  tmux kill-session -t epic   (또는 세션 이름)
set -u

S="${SESSION:-epic}"
D="/home/hmcl/stack1/execution"
SRC="source /opt/ros/noetic/setup.bash; source /home/hmcl/stack1/devel/setup.bash;"

AUTORUN=0
EPIC_ARG=""
FASTLIO_ARG=""
for a in "$@"; do
  case "$a" in
    --autorun)      AUTORUN=1 ;;
    --epic_rviz)    EPIC_ARG="--rviz" ;;
    --fastlio_rviz) FASTLIO_ARG="--rviz" ;;
    *) echo "[tmux] 알 수 없는 옵션: $a" ;;
  esac
done

# 이미 세션 있으면 그냥 attach
if tmux has-session -t "$S" 2>/dev/null; then
  echo "[tmux] 세션 '$S' 이미 있음 → attach (새로 만들려면: tmux kill-session -t $S)"
  exec tmux attach -t "$S"
fi

# 8 pane 생성 (split 후 매번 tiled 로 재배치)
tmux new-session -d -s "$S" -n stack
for _ in 1 2 3 4 5 6 7; do
  tmux split-window -t "$S":0
  tmux select-layout -t "$S":0 tiled
done

# pane 제목 표시 + 마우스 (버전에 따라 옵션명 다를 수 있어 실패 무시)
tmux setw -t "$S":0 pane-border-status top                          2>/dev/null || true
tmux setw -t "$S":0 pane-border-format " #{pane_index}:#{pane_title} " 2>/dev/null || true
tmux set  -t "$S"   mouse on                                        2>/dev/null || true

# send <pane> <title> <autorun_sleep> <command>
#   - 환경 source 는 항상 자동 실행
#   - command 는 --autorun 이면 자동 실행(앞에 sleep), 아니면 타이핑만 하고 Enter 대기
send() {
  local idx="$1" title="$2" slp="$3" cmd="$4"
  tmux select-pane -t "$S":0."$idx" -T "$title" 2>/dev/null || true
  tmux send-keys   -t "$S":0."$idx" "$SRC" C-m          # 환경 준비(자동)
  [ -z "$cmd" ] && return                                # 빈 셸 pane
  if [ "$AUTORUN" = "1" ]; then
    tmux send-keys -t "$S":0."$idx" "sleep $slp; $cmd" C-m   # 자동 실행
  else
    tmux send-keys -t "$S":0."$idx" "$cmd"                   # 타이핑만 (Enter 대기)
  fi
}

send 0 roscore   0  "if rostopic list >/dev/null 2>&1; then echo '[roscore] 기존 master 사용 (skip)'; else roscore; fi"
send 1 livox     3  "$D/1_livox.sh"
send 2 fastlio   8  "$D/2_fastlio.sh $FASTLIO_ARG"
send 3 mavros    5  "$D/3_mavros.sh"
send 4 tf_relay  14 "$D/4_tf_odom_relay.sh"
send 5 epic      18 "$D/5_epic.sh $EPIC_ARG"
send 6 monitor   20 "rostopic hz /Odometry /livox/lidar"
send 7 shell     0  ""

tmux select-pane -t "$S":0.0
if [ "$AUTORUN" = "1" ]; then
  echo "[tmux] --autorun: 전 pane 자동 실행 (sleep 스태거 적용)"
else
  echo "[tmux] 수동 모드: 각 pane 에 명령이 떠 있습니다. 0→1→2→3→4→5 순서로 Enter 치세요."
fi
exec tmux attach -t "$S"
