#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "Usage: $0 DETECTION_ROOT OUTPUT_ROOT [PLAYBACK_RATE]" >&2
  exit 2
fi

detection_root="$(realpath "$1")"
output_root="$(realpath -m "$2")"
playback_rate="${3:-1.0}"
repo_root="${AJOUNICE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"
runner="$repo_root/perception/detection3d/bevfusion/scripts/run_recorded_detections_pipeline_evaluation.sh"

if [[ "$playback_rate" != "1" && "$playback_rate" != "1.0" ]]; then
  echo "[WARN] The internal 30 Hz tracker scheduler is only reproducible at 1.0x playback." >&2
fi

# label|prediction_model|q_vx_vy|r_xy|r_yaw_deg|kinematic_projection
cases=(
  "baseline|1|1.0|0.3|10.0|false"
  "cv_baseline|0|1.0|0.3|10.0|false"
  "ctrv_rxy045|1|1.0|0.45|10.0|false"
  "ctrv_rxy060|1|1.0|0.6|10.0|false"
  "ctrv_qv060_rxy045|1|0.6|0.45|10.0|false"
  "ctrv_qv030_rxy060|1|0.3|0.6|10.0|false"
  "ctrv_kinematic|1|1.0|0.3|10.0|true"
)

bags=("vehicle_highway_merge" "vehicle_highway1")

raw_bag() {
  case "$1" in
    vehicle_highway_merge)
      printf '%s\n' "$repo_root/rosbags/raw_sensor_w_cam/perception_benchmark/260821_235409_highway_merge_dynamic_competition_raw.bag"
      ;;
    vehicle_highway1)
      printf '%s\n' "$repo_root/rosbags/raw_sensor_w_cam/sensor_cam_raw_highway1.bag"
      ;;
    *) return 2 ;;
  esac
}

mkdir -p "$output_root"
for spec in "${cases[@]}"; do
  IFS='|' read -r label model qv rxy ryaw kinematic <<<"$spec"
  for bag in "${bags[@]}"; do
    output="$output_root/$label/$bag"
    if [[ -s "$output/summary.json" && -s "$output/summary_cv_stable.json" ]]; then
      echo "[skip] $label/$bag already complete"
      continue
    fi
    detection_bag="$detection_root/$bag/detections.bag"
    test -s "$detection_bag"
    test -s "$(raw_bag "$bag")"
    echo "[run] $label/$bag model=$model qv=$qv rxy=$rxy ryaw=$ryaw kinematic=$kinematic"
    TRACKER_PREDICTION_MODEL="$model" \
    TRACKER_SYSTEM_NOISE_STD_VX_VY_MS="$qv" \
    TRACKER_MEAS_NOISE_STD_XY_M="$rxy" \
    TRACKER_MEAS_NOISE_STD_YAW_DEG="$ryaw" \
    TRACKER_USE_KINEMATIC_MODEL="$kinematic" \
      "$runner" "$(raw_bag "$bag")" "$detection_bag" "$output" \
      "ft-e7-$label-$bag" "$playback_rate"
  done
done

python3 "$repo_root/perception/detection3d/bevfusion/tools/summarize_highway_tracker_ablation.py" \
  "$output_root" --output-json "$output_root/summary.json" \
  --output-markdown "$output_root/summary.md"
