#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 3 || $# -gt 4 ]]; then
  echo "Usage: $0 RAW_BAG OUTPUT_DIR LABEL [PLAYBACK_RATE]" >&2
  exit 2
fi

raw_bag="$(realpath "$1")"
output_dir="$(realpath -m "$2")"
label="$3"
playback_rate="${4:-1.0}"
score_threshold="${BEVFUSION_EVAL_SCORE_THRESHOLD:-0.10}"
vehicle_score_threshold="${BEVFUSION_EVAL_VEHICLE_THRESHOLD:-0.40}"
pedestrian_score_threshold="${BEVFUSION_EVAL_PEDESTRIAN_THRESHOLD:-0.15}"
obstacle_score_threshold="${BEVFUSION_EVAL_OBSTACLE_THRESHOLD:-0.10}"
lidar_deskew_mode="${BEVFUSION_EVAL_LIDAR_DESKEW_MODE:-off}"
lidar_deskew_reference="${BEVFUSION_EVAL_LIDAR_DESKEW_REFERENCE:-end}"
lidar_deskew_pose_topic="${BEVFUSION_EVAL_LIDAR_DESKEW_POSE_TOPIC:-/Ego_topic}"
lidar_deskew_pose_type="${BEVFUSION_EVAL_LIDAR_DESKEW_POSE_TYPE:-ego_status}"
temporal_sweeps="${BEVFUSION_EVAL_TEMPORAL_SWEEPS:-0}"
temporal_maximum_age_sec="${BEVFUSION_EVAL_TEMPORAL_MAXIMUM_AGE_SEC:-0.30}"
publish_detector_velocity="${BEVFUSION_EVAL_PUBLISH_DETECTOR_VELOCITY:-false}"
detector_velocity_noise_std_mps="${BEVFUSION_EVAL_DETECTOR_VELOCITY_NOISE_STD_MPS:-1.0}"
gt_reference="${BEVFUSION_EVAL_GT_REFERENCE:-bbox-center-calibrated-v1}"
asset_bbox_catalog="${BEVFUSION_EVAL_ASSET_BBOX_CATALOG:-dataset/morai_od/configs/morai_asset_bbox_offsets_base3_v1.json}"
morai_heading_correction_deg="${BEVFUSION_EVAL_MORAI_HEADING_CORRECTION_DEG:-0.0}"
repo_root="${AJOUNICE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"
ros_port="${BEVFUSION_EVAL_ROS_PORT:-$((14000 + ($$ % 18000)))}"
export ROS_MASTER_URI="http://127.0.0.1:${ros_port}"
export ROS_IP=127.0.0.1
export ROS_HOME="/tmp/ajou_bevfusion_eval_ros_${ros_port}"

mkdir -p "$output_dir" "$ROS_HOME"
roscore_pid=""
bridge_pid=""
record_pid=""

cleanup() {
  set +e
  for pid in "$record_pid" "$bridge_pid" "$roscore_pid"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null
      wait "$pid" 2>/dev/null
    fi
  done
}
trap cleanup EXIT INT TERM

cd "$repo_root"
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash

roscore -p "$ros_port" >"$output_dir/roscore.log" 2>&1 &
roscore_pid=$!
for _ in $(seq 1 100); do
  rosparam list >/dev/null 2>&1 && break
  sleep 0.1
done
rosparam set /use_sim_time true

roslaunch bevfusion_ros base3_online.launch \
  start_rviz:=false \
  score_threshold:="$score_threshold" \
  vehicle_score_threshold:="$vehicle_score_threshold" \
  pedestrian_score_threshold:="$pedestrian_score_threshold" \
  obstacle_score_threshold:="$obstacle_score_threshold" \
  lidar_deskew_mode:="$lidar_deskew_mode" \
  lidar_deskew_reference:="$lidar_deskew_reference" \
  lidar_deskew_pose_topic:="$lidar_deskew_pose_topic" \
  lidar_deskew_pose_type:="$lidar_deskew_pose_type" \
  temporal_sweeps:="$temporal_sweeps" \
  temporal_maximum_age_sec:="$temporal_maximum_age_sec" \
  publish_detector_velocity:="$publish_detector_velocity" \
  detector_velocity_noise_std_mps:="$detector_velocity_noise_std_mps" \
  >"$output_dir/bevfusion_bridge.log" 2>&1 &
bridge_pid=$!

ready=false
for _ in $(seq 1 300); do
  if ! kill -0 "$bridge_pid" 2>/dev/null; then
    echo "[ERROR] BEVFusion ROS bridge exited during startup" >&2
    tail -n 100 "$output_dir/bevfusion_bridge.log" >&2
    exit 1
  fi
  if rostopic info /perception/object_recognition/detection/objects \
      2>/dev/null | grep -q '/bevfusion_ros_bridge'; then
    ready=true
    break
  fi
  sleep 0.1
done
if [[ "$ready" != true ]]; then
  echo "[ERROR] BEVFusion detection publisher did not become ready" >&2
  exit 1
fi

rosbag record --lz4 --buffsize=1024 \
  -O "$output_dir/detections.bag" \
  /perception/object_recognition/detection/objects \
  /perception/bevfusion/latency_ms \
  /perception/bevfusion/status \
  >"$output_dir/record.log" 2>&1 &
record_pid=$!
sleep 0.5

echo "[play] $label: $raw_bag"
rosbag play --quiet --clock --rate="$playback_rate" "$raw_bag" \
  >"$output_dir/rosbag.log" 2>&1
sleep 2

kill -INT "$record_pid" 2>/dev/null || true
wait "$record_pid" || true
record_pid=""
kill -INT "$bridge_pid" 2>/dev/null || true
wait "$bridge_pid" || true
bridge_pid=""

/usr/bin/python3 \
  base_ws/src/tracking_prediction_evaluation/scripts/evaluate_rosbag_detection.py \
  "$raw_bag" "$output_dir/detections.bag" \
  --output "$output_dir/summary.json" \
  --gt-reference "$gt_reference" \
  --asset-bbox-catalog "$asset_bbox_catalog" \
  --morai-heading-correction-deg "$morai_heading_correction_deg"

/usr/bin/python3 - "$output_dir/summary.json" \
  "$score_threshold" "$vehicle_score_threshold" \
  "$pedestrian_score_threshold" "$obstacle_score_threshold" \
  "$lidar_deskew_mode" "$lidar_deskew_reference" \
  "$lidar_deskew_pose_topic" "$lidar_deskew_pose_type" \
  "$temporal_sweeps" "$temporal_maximum_age_sec" \
  "$publish_detector_velocity" "$detector_velocity_noise_std_mps" <<'PY'
import json
import sys

path = sys.argv[1]
with open(path, encoding="utf-8") as handle:
    report = json.load(handle)
report["runtime_class_score_thresholds"] = {
    "fallback": float(sys.argv[2]),
    "vehicle": float(sys.argv[3]),
    "pedestrian": float(sys.argv[4]),
    "obstacle": float(sys.argv[5]),
}
report["lidar_deskew"] = {
    "mode": sys.argv[6],
    "reference": sys.argv[7],
    "pose_topic": sys.argv[8],
    "pose_type": sys.argv[9],
}
report["temporal_input"] = {
    "past_sweeps": int(sys.argv[10]),
    "maximum_age_sec": float(sys.argv[11]),
}
report["detector_velocity_output"] = {
    "enabled": sys.argv[12].strip().lower() == "true",
    "noise_std_mps": float(sys.argv[13]),
}
with open(path, "w", encoding="utf-8") as handle:
    json.dump(report, handle, indent=2, sort_keys=True)
    handle.write("\n")
PY

echo "[evaluation] $label -> $output_dir/summary.json"
