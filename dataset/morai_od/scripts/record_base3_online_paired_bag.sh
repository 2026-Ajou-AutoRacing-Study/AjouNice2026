#!/usr/bin/env bash
set -euo pipefail

ROOT="${AJOUNICE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}"
OUT_ROOT="${1:-$ROOT/rosbags/datasets/base3_online/paired_pilot}"
RUN_TAG="${RUN_TAG:-$(date +%y%m%d_%H%M%S)}"
PREFIX="$OUT_ROOT/${RUN_TAG}_base3_online_paired"
INCLUDE_CLOCK="${INCLUDE_CLOCK:-0}"
GT_TRANSPORT="${GT_TRANSPORT:-ros}"

mkdir -p "$OUT_ROOT"
source /opt/ros/noetic/setup.bash
if [[ -f "$ROOT/base_ws/install_aw149/setup.bash" ]]; then
  source "$ROOT/base_ws/install_aw149/setup.bash"
else
  source "$ROOT/base_ws/devel/setup.bash"
fi
if [[ -f "$HOME/morai_personal_ws/devel/setup.bash" ]]; then
  source "$HOME/morai_personal_ws/devel/setup.bash"
fi

TOPICS=(
  /sensor/lidar1
  /sensor/camera/front/compressed
  /sensor/camera/front_left/compressed
  /sensor/camera/front_right/compressed
  /sensor/camera/back/compressed
  /Ego_topic
  /Object_topic
  /sensor/gps
  /sensor/imu
  /CollisionData
  /VehicleCollisionData
  /sim/process/state/msg/MoraiSimProcStatus
  /tf
  /tf_static
  /dataset/scenario_marker
)
if [[ "$INCLUDE_CLOCK" == "1" ]]; then
  TOPICS+=(/clock)
fi

python3 "$ROOT/dataset/morai_od/scripts/check_base3_online_runtime.py" \
  --gt-transport "$GT_TRANSPORT"

{
  echo "run_tag=$RUN_TAG"
  echo "started_at=$(date --iso-8601=seconds)"
  echo "git_sha=$(git -C "$ROOT" rev-parse HEAD)"
  echo "include_clock=$INCLUDE_CLOCK"
  echo "gt_transport=$GT_TRANSPORT"
  printf 'topic=%s\n' "${TOPICS[@]}"
} > "${PREFIX}_recording_contract.txt"

echo "Recording Base3-Online paired pilot to ${PREFIX}_*.bag"
echo "Stop with Ctrl-C only after the scenario runner has finished."
exec rosbag record --lz4 --split --size=4096 -O "$PREFIX" "${TOPICS[@]}"
