#!/usr/bin/env bash

# Record continuous online sensor, GT, and runtime streams for Base4-Temporal.
# SaveSensorData is intentionally not part of this contract.

set -Eeuo pipefail

ROOT="${AJOUNICE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}"
OUT_ROOT="${1:-${ROOT}/rosbags/datasets/base4_temporal/pilot}"
RUN_TAG="${RUN_TAG:-$(date +%y%m%d_%H%M%S)_base4_temporal}"
PREFIX="${OUT_ROOT}/${RUN_TAG}"
GT_TRANSPORT="${GT_TRANSPORT:-ros}"
SENSOR_CONFIG="${SENSOR_CONFIG:-${ROOT}/configs/sensors/Sensor_competition_local.json}"
NETWORK_CONFIG="${NETWORK_CONFIG:-${ROOT}/configs/network/ego_network_mr_minimal_hybrid.json}"

REQUIRED_TOPICS=(
  /sensor/lidar1
  /sensor/camera/front/compressed
  /sensor/camera/front_left/compressed
  /sensor/camera/front_right/compressed
  /sensor/camera/back/compressed
  /Ego_topic
  /Object_topic
  /dataset/scenario_marker
  /ctrl_cmd
  /sensor/gps
  /sensor/imu
)

# These streams are useful for diagnosing invalid clips, but the minimal
# five-interface Ego Network profile does not promise to advertise them.
OPTIONAL_TOPICS=(
  /CollisionData
  /VehicleCollisionData
  /sim/process/state/msg/MoraiSimProcStatus
  /tf
  /tf_static
)

mkdir -p "${OUT_ROOT}"
source /opt/ros/noetic/setup.bash
if [[ -f "${ROOT}/base_ws/install_aw149/setup.bash" ]]; then
  source "${ROOT}/base_ws/install_aw149/setup.bash"
else
  source "${ROOT}/base_ws/devel/setup.bash"
fi

python3 "${ROOT}/dataset/morai_od/scripts/check_base3_online_runtime.py" \
  --gt-transport "${GT_TRANSPORT}"

sha256_or_missing() {
  if [[ -f "$1" ]]; then
    sha256sum "$1" | awk '{print $1}'
  else
    printf 'missing'
  fi
}

{
  echo "schema_version=1"
  echo "dataset_family=base4_temporal"
  echo "capture_mode=continuous_rosbag"
  echo "save_sensor_data=false"
  echo "run_tag=${RUN_TAG}"
  echo "started_at=$(date --iso-8601=seconds)"
  echo "git_sha=$(git -C "${ROOT}" rev-parse HEAD)"
  echo "gt_transport=${GT_TRANSPORT}"
  echo "sensor_config=${SENSOR_CONFIG}"
  echo "sensor_config_sha256=$(sha256_or_missing "${SENSOR_CONFIG}")"
  echo "network_config=${NETWORK_CONFIG}"
  echo "network_config_sha256=$(sha256_or_missing "${NETWORK_CONFIG}")"
  printf 'required_topic=%s\n' "${REQUIRED_TOPICS[@]}"
  printf 'optional_topic=%s\n' "${OPTIONAL_TOPICS[@]}"
} > "${PREFIX}_recording_contract.txt"

echo "Recording Base4-Temporal continuous streams to ${PREFIX}_*.bag"
echo "Start the scenario runner only after this command reports subscription setup."
echo "Stop with Ctrl-C only after the runner has emitted the final capture_end marker."
exec rosbag record \
  --buffsize=4096 \
  --lz4 \
  --split \
  --size=4096 \
  -O "${PREFIX}" \
  "${REQUIRED_TOPICS[@]}" \
  "${OPTIONAL_TOPICS[@]}"
