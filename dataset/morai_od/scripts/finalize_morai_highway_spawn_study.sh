#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 3 ]]; then
  echo "usage: $0 <rosbag-pid> <bag-path> <output-dir>" >&2
  exit 2
fi

RECORDER_PID="$1"
BAG_PATH="$2"
OUTPUT_DIR="$3"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

while kill -0 "${RECORDER_PID}" 2>/dev/null; do
  sleep 10
done

for _ in $(seq 1 30); do
  [[ -f "${BAG_PATH}" ]] && break
  sleep 1
done
if [[ ! -f "${BAG_PATH}" ]]; then
  echo "finalized bag not found: ${BAG_PATH}" >&2
  exit 1
fi

mkdir -p "${OUTPUT_DIR}"
source /opt/ros/noetic/setup.bash
source /home/luv/morai_personal_ws/devel/setup.bash
cd "${ROOT}"
python3 dataset/morai_od/scripts/analyze_morai_highway_random_spawn.py \
  "${BAG_PATH}" \
  --output-json "${OUTPUT_DIR}/spawn_statistics.json" \
  --output-md "${OUTPUT_DIR}/spawn_statistics.md"
rosbag info --yaml "${BAG_PATH}" > "${OUTPUT_DIR}/rosbag_info.yaml"
date --iso-8601=seconds > "${OUTPUT_DIR}/COMPLETED"
echo "spawn study finalized: ${OUTPUT_DIR}"
