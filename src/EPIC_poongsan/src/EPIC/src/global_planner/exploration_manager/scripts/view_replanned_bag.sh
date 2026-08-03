#!/usr/bin/env bash
set -eo pipefail

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPOSITORY_DIR="$(git -C "$PACKAGE_DIR" rev-parse --show-toplevel 2>/dev/null || true)"

usage() {
  cat >&2 <<EOF
Usage: $0 BAG_PATH [RATE] [START] [DURATION] [RVIZ] [CONFIG_FILE]

Examples:
  $0 /workspace/sim_validation_ws/result/real_flight/0803/flight.bag
  $0 /workspace/sim_validation_ws/result/real_flight/0803/flight.bag 1.0 30 15 false real2.yaml

Environment:
  EPIC_REPLAY_SETUP  Current workspace setup.bash. Set this when the repository
                     was built outside its default devel directory.
EOF
  exit 2
}

[[ $# -ge 1 ]] || usage

BAG_PATH="$1"
RATE="${2:-1.0}"
START="${3:-0.0}"
DURATION="${4:-}"
RVIZ="${5:-true}"
CONFIG_FILE="${6:-real2.yaml}"

if [[ ! -r "$BAG_PATH" ]]; then
  echo "[view_replanned_bag] Bag is not readable: $BAG_PATH" >&2
  exit 2
fi

source /opt/ros/noetic/setup.bash

SETUP_USED=""
if [[ -n "${EPIC_REPLAY_SETUP:-}" ]]; then
  if [[ ! -r "$EPIC_REPLAY_SETUP" ]]; then
    echo "[view_replanned_bag] EPIC_REPLAY_SETUP is not readable: $EPIC_REPLAY_SETUP" >&2
    exit 2
  fi
  source "$EPIC_REPLAY_SETUP"
  SETUP_USED="$EPIC_REPLAY_SETUP"
elif [[ -n "$REPOSITORY_DIR" && -r "$REPOSITORY_DIR/devel/setup.bash" ]]; then
  source "$REPOSITORY_DIR/devel/setup.bash"
  SETUP_USED="$REPOSITORY_DIR/devel/setup.bash"
else
  # Validation builds may use an isolated /tmp/*_devel space. Select the
  # newest setup whose epic_planner package resolves to this source tree.
  while IFS= read -r setup_file; do
    source "$setup_file"
    resolved_package="$(rospack find epic_planner 2>/dev/null || true)"
    if [[ -n "$resolved_package" &&
          "$(readlink -f "$resolved_package")" == "$(readlink -f "$PACKAGE_DIR")" ]]; then
      SETUP_USED="$setup_file"
      break
    fi
  done < <(find /tmp -maxdepth 2 -path '*_devel/setup.bash' \
           -printf '%T@ %p\n' 2>/dev/null | sort -nr | cut -d' ' -f2-)
fi

RESOLVED_PACKAGE="$(rospack find epic_planner 2>/dev/null || true)"
if [[ -z "$RESOLVED_PACKAGE" ||
      "$(readlink -f "$RESOLVED_PACKAGE")" != "$(readlink -f "$PACKAGE_DIR")" ]]; then
  echo "[view_replanned_bag] The current repository build is not sourced." >&2
  echo "Set EPIC_REPLAY_SETUP=/path/to/current/devel/setup.bash and retry." >&2
  exit 3
fi

if rosnode list 2>/dev/null | grep -Eq '^/(exploration_node|bag_replay)$'; then
  echo "[view_replanned_bag] Another planner/replay is running. Stop it before replay." >&2
  exit 4
fi

if [[ "$RVIZ" == "true" || "$RVIZ" == "1" ]]; then
  export DISPLAY="${DISPLAY:-:1}"
  export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
fi

COMMIT="$(git -C "$PACKAGE_DIR" rev-parse --short HEAD 2>/dev/null || echo unknown)"
echo "[view_replanned_bag] source : $PACKAGE_DIR"
echo "[view_replanned_bag] commit : $COMMIT"
echo "[view_replanned_bag] setup  : ${SETUP_USED:-already sourced environment}"
echo "[view_replanned_bag] config : $CONFIG_FILE"
echo "[view_replanned_bag] bag    : $BAG_PATH"
echo "[view_replanned_bag] rate   : $RATE"

exec roslaunch epic_planner bag_replay_current.launch \
  bag:="$BAG_PATH" \
  config_file:="$CONFIG_FILE" \
  rate:="$RATE" \
  start:="$START" \
  duration:="$DURATION" \
  rviz:="$RVIZ"
