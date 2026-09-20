#!/usr/bin/env bash

# Keep a bounded in-memory rosbag ring buffer and flush one bag only after a
# scenario runner emits a successful capture_end marker.

set -Eeuo pipefail

ROOT="${AJOUNICE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}"
OUT_ROOT="${1:-${ROOT}/rosbags/datasets/base4_temporal/gated_pilot}"
RUN_TAG="${RUN_TAG:-$(date +%y%m%d_%H%M%S)_base4_temporal_gated}"
PREFIX="${OUT_ROOT}/${RUN_TAG}"
BUFFER_MB="${BUFFER_MB:-48}"
POST_ROLL_SEC="${POST_ROLL_SEC:-0.25}"
MAX_WINDOW_SEC="${MAX_WINDOW_SEC:-3.40}"
MIN_SPACE="${MIN_SPACE:-35G}"
GT_TRANSPORT="${GT_TRANSPORT:-ros}"
SENSOR_CONFIG="${SENSOR_CONFIG:-${ROOT}/configs/sensors/Sensor_competition_local.json}"
NETWORK_CONFIG="${NETWORK_CONFIG:-${ROOT}/configs/network/ego_network_mr_minimal_hybrid.json}"
EVENT_LOG="${PREFIX}_snapshot_events.jsonl"

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

if rostopic info /snapshot_trigger 2>/dev/null | grep -q '/record_'; then
  echo "another rosbag snapshot recorder is already subscribed to /snapshot_trigger" >&2
  exit 1
fi
if rosnode list 2>/dev/null | grep -qx '/base4_rosbag_snapshot_gate'; then
  echo "another Base4 snapshot gate is already running" >&2
  exit 1
fi

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
  echo "capture_mode=snapshot_gated_rosbag"
  echo "save_sensor_data=false"
  echo "run_tag=${RUN_TAG}"
  echo "started_at=$(date --iso-8601=seconds)"
  echo "git_sha=$(git -C "${ROOT}" rev-parse HEAD)"
  echo "gt_transport=${GT_TRANSPORT}"
  echo "snapshot_buffer_mb=${BUFFER_MB}"
  echo "post_roll_sec=${POST_ROLL_SEC}"
  echo "max_window_sec=${MAX_WINDOW_SEC}"
  echo "min_space=${MIN_SPACE}"
  echo "sensor_config=${SENSOR_CONFIG}"
  echo "sensor_config_sha256=$(sha256_or_missing "${SENSOR_CONFIG}")"
  echo "network_config=${NETWORK_CONFIG}"
  echo "network_config_sha256=$(sha256_or_missing "${NETWORK_CONFIG}")"
  printf 'required_topic=%s\n' "${REQUIRED_TOPICS[@]}"
  printf 'optional_topic=%s\n' "${OPTIONAL_TOPICS[@]}"
} > "${PREFIX}_recording_contract.txt"

RECORDER_PID=""
GATE_PID=""
cleanup() {
  trap - INT TERM EXIT
  if [[ -n "${GATE_PID}" ]] && kill -0 "${GATE_PID}" 2>/dev/null; then
    kill -INT "${GATE_PID}" 2>/dev/null || true
    wait "${GATE_PID}" 2>/dev/null || true
  fi
  if [[ -n "${RECORDER_PID}" ]] && kill -0 "${RECORDER_PID}" 2>/dev/null; then
    kill -INT "${RECORDER_PID}" 2>/dev/null || true
    wait "${RECORDER_PID}" 2>/dev/null || true
  fi
}
trap cleanup INT TERM EXIT

TOPIC_ARGS=()
for topic in "${REQUIRED_TOPICS[@]}" "${OPTIONAL_TOPICS[@]}"; do
  TOPIC_ARGS+=(--topic "${topic}")
done

/opt/ros/noetic/lib/rosbag/record \
  --snapshot \
  --buffsize "${BUFFER_MB}" \
  --lz4 \
  --min-space "${MIN_SPACE}" \
  --output-prefix "${PREFIX}" \
  "${TOPIC_ARGS[@]}" &
RECORDER_PID=$!

for _ in $(seq 1 50); do
  if rostopic info /snapshot_trigger 2>/dev/null | grep -q '/record_'; then
    break
  fi
  if ! kill -0 "${RECORDER_PID}" 2>/dev/null; then
    echo "rosbag snapshot recorder exited during startup" >&2
    exit 1
  fi
  sleep 0.1
done

if ! rostopic info /snapshot_trigger 2>/dev/null | grep -q '/record_'; then
  echo "rosbag snapshot recorder did not subscribe to /snapshot_trigger" >&2
  exit 1
fi

python3 "${ROOT}/dataset/morai_od/scripts/gate_base4_rosbag_snapshots.py" \
  --event-log "${EVENT_LOG}" \
  --bag-prefix "${PREFIX}" \
  --post-roll-sec "${POST_ROLL_SEC}" \
  --max-window-sec "${MAX_WINDOW_SEC}" &
GATE_PID=$!

echo "Base4 gated recorder ready."
echo "  output prefix : ${PREFIX}"
echo "  ring buffer   : ${BUFFER_MB} MB"
echo "  event log     : ${EVENT_LOG}"
echo "Start the scenario runner now; stop this command only after the final snapshot closes."

wait "${GATE_PID}"
