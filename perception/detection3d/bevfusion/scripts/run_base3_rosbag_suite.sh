#!/usr/bin/env bash
set -euo pipefail

repo_root="${AJOUNICE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"
output_root="${1:-$repo_root/artifacts/evaluation/bevfusion/base3_mission_rosbag_suite}"
playback_rate_override="${2:-}"
runner="$repo_root/perception/detection3d/bevfusion/scripts/run_rosbag_detection_evaluation.sh"

specs=(
  "vehicle_roundabout_new|rosbags/raw_sensor_w_cam/perception_benchmark/260821_235243_roundabout_dynamic_competition_raw.bag|1.0|-1.347"
  "vehicle_highway_merge|rosbags/raw_sensor_w_cam/perception_benchmark/260821_235409_highway_merge_dynamic_competition_raw.bag|1.0|-1.347"
  "vehicle_highway1|rosbags/raw_sensor_w_cam/sensor_cam_raw_highway1.bag|0.5|-1.347"
  # Static-Ego pedestrian bags cannot be motion-audited. Keep their historical
  # 0-degree contract until launcher provenance is recovered.
  "pedestrian_crossing|rosbags/raw_sensor_w_cam/pointpainting_eval_260730/pointpainting_pedestrian_crossing_raw.bag|1.0|0.0"
  "pedestrian_distance|rosbags/raw_sensor_w_cam/pointpainting_eval_260730/pointpainting_pedestrian_distance_5_10_15_18m_raw.bag|1.0|0.0"
  "obstacle_backforth|rosbags/raw_sensor_w_cam/perception_benchmark/260912_165936_obstacle_backforth_mr.bag|1.0|-1.347"
  "obstacle_base3_multiasset|rosbags/raw_sensor_w_cam/perception_benchmark/260912_195556_bench_static_obstacle_base3_assets_raw.bag|0.25|-1.347"
)

mkdir -p "$output_root"
for spec in "${specs[@]}"; do
  IFS='|' read -r label relative_bag recommended_rate heading_correction_deg <<<"$spec"
  playback_rate="${playback_rate_override:-$recommended_rate}"
  summary="$output_root/$label/summary.json"
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
  echo "[start] $label rate=$playback_rate: $relative_bag"
  BEVFUSION_EVAL_MORAI_HEADING_CORRECTION_DEG="$heading_correction_deg" \
    bash "$runner" \
    "$repo_root/$relative_bag" \
    "$output_root/$label" \
    "$label" \
    "$playback_rate"
done

echo "[complete] $output_root"
