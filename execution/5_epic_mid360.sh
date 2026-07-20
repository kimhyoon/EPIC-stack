#!/usr/bin/env bash
# Run the Git-distributed MID360 EPIC profiles without a machine-specific path.
# 회피/crop 의 on/off 는 스크립트/launch 인자가 아니라 선택된 profile yaml 안의
# local_avoidance/enable, cloud_crop/enable 값이 단일 소스다 (false = 해당 노드 자진 종료).
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
CONFIG_FILE="mid360_mlx.yaml"
USE_RVIZ="true"

usage() {
  cat <<'EOF'
Usage: ./execution/5_epic_mid360.sh [--raw|--mlx-crop] [--rviz|--no-rviz]

  no option   Run the MID360-to-ML-X profile (config/mid360_mlx.yaml).
  --mlx-crop  Same as default, kept for explicitness.
  --raw       Run the raw MID360 debug profile (config/mid360.yaml).
  --rviz      Open the inspection RViz alongside the planner (default: on).
  --no-rviz   Planner only.

회피/crop 의 on/off 는 여기 인자가 아니라 선택된 profile yaml 안의
local_avoidance/enable, cloud_crop/enable 값이 단일 소스다 (false = 해당 노드 자진 종료).

RViz can also be started separately:
  roslaunch ml_x_cropping mid360_epic_rviz.launch
EOF
}

for arg in "$@"; do
  case "${arg}" in
    --mlx-crop)
      CONFIG_FILE="mid360_mlx.yaml"
      ;;
    --raw)
      CONFIG_FILE="mid360.yaml"
      ;;
    --rviz)
      USE_RVIZ="true"
      ;;
    --no-rviz)
      USE_RVIZ="false"
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: ${arg}" >&2
      usage >&2
      exit 2
      ;;
  esac
done

source /opt/ros/noetic/setup.bash

if [[ ! -f "${WORKSPACE_ROOT}/devel/setup.bash" ]]; then
  echo "Missing ${WORKSPACE_ROOT}/devel/setup.bash. Build this workspace first." >&2
  exit 1
fi
source "${WORKSPACE_ROOT}/devel/setup.bash"

# crop 브릿지는 항상 launch 되고 yaml(cloud_crop/enable)이 자진 종료를 결정하므로
# 바이너리는 어느 프로파일이든 필요하다.
if [[ ! -x "${WORKSPACE_ROOT}/devel/lib/ml_x_cropping/cloud_crop_bridge" ]]; then
  echo "cloud_crop_bridge is not built. Rebuild with BUILD_CLOUD_CROP_BRIDGE=ON." >&2
  exit 1
fi

echo "[epic] profile=${CONFIG_FILE} rviz=${USE_RVIZ} (회피/crop 스위치는 profile yaml)"
LAUNCH_ARGS=(
  config_file:="${CONFIG_FILE}"
)

if [[ "${USE_RVIZ}" == "true" ]]; then
  # rviz 를 백그라운드로 함께 띄우고, 플래너 종료(Ctrl+C 포함) 시 같이 정리한다.
  roslaunch ml_x_cropping mid360_epic_rviz.launch & RVIZ_PID=$!
  trap 'kill -INT "${RVIZ_PID}" 2>/dev/null; wait "${RVIZ_PID}" 2>/dev/null' EXIT
  roslaunch epic_planner real_flight.launch "${LAUNCH_ARGS[@]}"
else
  exec roslaunch epic_planner real_flight.launch "${LAUNCH_ARGS[@]}"
fi
