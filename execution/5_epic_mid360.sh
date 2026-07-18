#!/usr/bin/env bash
# Run the Git-distributed MID360 EPIC profiles without a machine-specific path.
# Default: raw MID360 debug baseline. --mlx-crop: one ML-X emulation crop pass.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
CONFIG_FILE="mid360.yaml"
USE_CROP="false"

usage() {
  cat <<'EOF'
Usage: ./execution/5_epic_mid360.sh [--mlx-crop]

  no option   Run the raw MID360 debug profile. EPIC consumes /cloud_registered.
  --mlx-crop  Run the MID360-to-ML-X profile. The bridge publishes one cropped
              cloud on /cloud_registered_cropped for EPIC.

RViz is started separately:
  roslaunch ml_x_cropping mid360_epic_rviz.launch
EOF
}

for arg in "$@"; do
  case "${arg}" in
    --mlx-crop)
      CONFIG_FILE="mid360_mlx.yaml"
      USE_CROP="true"
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

if [[ "${USE_CROP}" == "true" && \
      ! -x "${WORKSPACE_ROOT}/devel/lib/ml_x_cropping/cloud_crop_bridge" ]]; then
  echo "cloud_crop_bridge is not built. Rebuild with BUILD_CLOUD_CROP_BRIDGE=ON." >&2
  exit 1
fi

echo "[epic] profile=${CONFIG_FILE} use_cloud_crop_bridge=${USE_CROP}"
exec roslaunch epic_planner real_flight.launch \
  config_file:="${CONFIG_FILE}" \
  use_cloud_crop_bridge:="${USE_CROP}"
