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
  score_threshold:=0.01 \
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
  --output "$output_dir/summary.json"

echo "[evaluation] $label -> $output_dir/summary.json"
