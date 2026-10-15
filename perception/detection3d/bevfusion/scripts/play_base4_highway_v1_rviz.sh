#!/usr/bin/env bash
set -euo pipefail

if [[ $# -gt 1 ]]; then
  echo "Usage: $0 [RATE]" >&2
  exit 2
fi

rate="${1:-0.25}"
root="${AJOUNICE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"
raw="$root/rosbags/raw_sensor_w_cam/sensor_cam_raw_highway1.bag"
detections="$root/artifacts/evaluation/bevfusion/base4_hwy3h1_mrhc_v2_v1e4_velocity_highway_260921/vehicle_highway1/detections.bag"
catalog="$root/dataset/morai_od/configs/morai_asset_bbox_offsets_base3_v1.json"

for path in "$raw" "$detections" "$catalog"; do
  [[ -s "$path" ]] || { echo "Missing required input: $path" >&2; exit 1; }
done

port="${BASE4_HIGHWAY_RVIZ_ROS_PORT:-$((16000 + ($$ % 12000)))}"
export AJOUNICE_ROOT="$root"

source /opt/ros/noetic/setup.bash
# The prediction node imports Autoware ROS 2 compatibility messages from the
# AW149 overlay. Source it first, then use the ROS 1 devel overlay so this
# checkout's visualization script and launch changes are picked up directly.
source "$root/scripts/workspace/source_aw149.sh"
aw149_pythonpath="${PYTHONPATH:-$root/base_ws/install_aw149/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages}"
aw149_ld_library_path="${LD_LIBRARY_PATH:-}"
source "$root/base_ws/devel/setup.bash"
export ROS_MASTER_URI="http://127.0.0.1:${port}"
export ROS_IP=127.0.0.1
export ROS_HOME="/tmp/ajou_base4_highway_v1_rviz_${port}"
mkdir -p "$ROS_HOME"

roscore_pid=""
pipeline_pid=""
rviz_pid=""
cleanup() {
  set +e
  for pid in "$rviz_pid" "$pipeline_pid" "$roscore_pid"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null
      wait "$pid" 2>/dev/null
    fi
  done
}
trap cleanup EXIT INT TERM

roscore -p "$port" >"$ROS_HOME/roscore.log" 2>&1 &
roscore_pid=$!
for _ in $(seq 1 100); do
  rosparam list >/dev/null 2>&1 && break
  sleep 0.1
done
rosparam set /use_sim_time true

env PYTHONPATH="$aw149_pythonpath" LD_LIBRARY_PATH="$aw149_ld_library_path" \
  PYTHONNOUSERSITE=1 \
  /usr/bin/python3 /opt/ros/noetic/bin/roslaunch \
  perception_object_pipeline tracking_prediction.launch \
  use_sim_time:=true \
  tracker_detection_velocity_fusion_mode:=0 \
  >"$ROS_HOME/tracking_prediction.log" 2>&1 &
pipeline_pid=$!

for _ in $(seq 1 300); do
  if ! kill -0 "$pipeline_pid" 2>/dev/null; then
    tail -n 100 "$ROS_HOME/tracking_prediction.log" >&2
    exit 1
  fi
  rosnode list 2>/dev/null | grep -q '^/map_based_prediction$' && break
  sleep 0.1
done

env PYTHONPATH="$aw149_pythonpath" LD_LIBRARY_PATH="$aw149_ld_library_path" \
  PYTHONNOUSERSITE=1 \
  /usr/bin/python3 /opt/ros/noetic/bin/roslaunch \
  tracking_prediction_evaluation pipeline_prediction_rviz.launch \
  use_sim_time:=true \
  start_rviz:=true \
  show_gt_boxes:=true \
  use_heading_correction:=true \
  morai_heading_correction_deg:=-1.347 \
  asset_bbox_catalog:="$catalog" \
  >"$ROS_HOME/rviz.log" 2>&1 &
rviz_pid=$!

for _ in $(seq 1 300); do
  if ! kill -0 "$rviz_pid" 2>/dev/null; then
    tail -n 100 "$ROS_HOME/rviz.log" >&2
    exit 1
  fi
  rosnode list 2>/dev/null | grep -q '^/morai_gt_bbox_visualization$' && break
  sleep 0.1
done

echo "============================================================"
echo "Base4 corrected V1 E4 / highway1"
echo "Playback rate: $rate"
echo "Vehicle threshold: 0.40; detector velocity fusion: OFF"
echo "Magenta: audited asset-catalog BBox center and size"
echo "GT correction: RandomMode vehicle size match + MR heading -1.347 deg"
echo "SPACE: play/pause, Ctrl-C: close"
echo "Logs: $ROS_HOME"
echo "============================================================"

rosbag play --clock --pause --keep-alive --rate="$rate" \
  "$raw" "$detections"
