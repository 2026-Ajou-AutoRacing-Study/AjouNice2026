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
  "vehicle_roundabout_new|rosbags/raw_sensor_w_cam/perception_benchmark/260821_235243_roundabout_dynamic_competition_raw.bag|1.0|-1.347"
  "vehicle_highway_merge|rosbags/raw_sensor_w_cam/perception_benchmark/260821_235409_highway_merge_dynamic_competition_raw.bag|1.0|-1.347"
  "vehicle_highway1|rosbags/raw_sensor_w_cam/sensor_cam_raw_highway1.bag|0.5|-1.347"
)

mkdir -p "$output_root"
for spec in "${specs[@]}"; do
  IFS='|' read -r label relative_bag recommended_rate heading_correction_deg <<<"$spec"
  rate="${rate_override:-$recommended_rate}"
  detections="$detection_root/$label/detections.bag"
  summary="$output_root/$label/summary.json"
  test -s "$detections" || {
    echo "[ERROR] detections bag is missing: $detections" >&2
    exit 1
  }
  if [[ -s "$summary" ]] && /usr/bin/python3 - "$summary" "$heading_correction_deg" <<'PY'
import json
import math
import sys
with open(sys.argv[1], encoding="utf-8") as handle:
    report = json.load(handle)
actual = report.get("source", {}).get("morai_heading_correction_deg")
raise SystemExit(0 if actual is not None and math.isclose(float(actual), float(sys.argv[2]), abs_tol=1e-9) else 1)
PY
  then
    echo "[skip] $label already completed with heading=$heading_correction_deg: $summary"
    continue
  fi
  BEVFUSION_EVAL_MORAI_HEADING_CORRECTION_DEG="$heading_correction_deg" \
    bash "$runner" "$repo_root/$relative_bag" "$detections" \
      "$output_root/$label" "$label" "$rate"
done

echo "[complete] $output_root"
