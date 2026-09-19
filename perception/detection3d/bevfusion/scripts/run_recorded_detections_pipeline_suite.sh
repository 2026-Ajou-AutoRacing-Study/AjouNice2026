#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "Usage: $0 DETECTION_ROOT OUTPUT_ROOT [PLAYBACK_RATE_OVERRIDE]" >&2
  exit 2
fi

repo_root="${AJOUNICE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"
detection_root="$(realpath "$1")"
output_root="$(realpath -m "$2")"
rate_override="${3:-}"
runner="$repo_root/perception/detection3d/bevfusion/scripts/run_recorded_detections_pipeline_evaluation.sh"

specs=(
  "vehicle_roundabout_new|rosbags/raw_sensor_w_cam/perception_benchmark/260821_235243_roundabout_dynamic_competition_raw.bag|1.0"
  "vehicle_highway_merge|rosbags/raw_sensor_w_cam/perception_benchmark/260821_235409_highway_merge_dynamic_competition_raw.bag|1.0"
  "vehicle_highway1|rosbags/raw_sensor_w_cam/sensor_cam_raw_highway1.bag|0.5"
)

mkdir -p "$output_root"
for spec in "${specs[@]}"; do
  IFS='|' read -r label relative_bag recommended_rate <<<"$spec"
  rate="${rate_override:-$recommended_rate}"
  detections="$detection_root/$label/detections.bag"
  summary="$output_root/$label/summary.json"
  test -s "$detections" || {
    echo "[ERROR] detections bag is missing: $detections" >&2
    exit 1
  }
  if [[ -s "$summary" ]]; then
    echo "[skip] $label already completed: $summary"
    continue
  fi
  bash "$runner" "$repo_root/$relative_bag" "$detections" \
    "$output_root/$label" "$label" "$rate"
done

echo "[complete] $output_root"
