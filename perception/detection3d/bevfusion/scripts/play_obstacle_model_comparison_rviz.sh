#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "Usage: $0 {pp1|e10} {multiasset|backforth} [RATE]" >&2
  exit 2
fi

model="$1"
scene="$2"
rate="${3:-1.0}"
root="${AJOUNICE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"

case "$scene" in
  multiasset)
    raw="$root/artifacts/evaluation/pointpainting/260912_base3_obstacle_benchmark/input_trim6s.bag"
    case "$model" in
      pp1)
        detections="$root/artifacts/evaluation/pointpainting/260912_base3_obstacle_benchmark/pp1_e34/detections_thresholds_v040_p015_o010.bag"
        ;;
      e10)
        detections="$root/artifacts/evaluation/bevfusion/base3_b0_e10_class_thresholds_040_015_010_260913/obstacle_base3_multiasset/detections_trim6s.bag"
        ;;
      *) echo "Unknown model: $model" >&2; exit 2 ;;
    esac
    ;;
  backforth)
    raw="$root/rosbags/raw_sensor_w_cam/perception_benchmark/260912_165936_obstacle_backforth_mr.bag"
    case "$model" in
      pp1)
        detections="$root/artifacts/evaluation/pointpainting/260912_pp1_obstacle_backforth_mr_class_thresholds/detections.bag"
        ;;
      e10)
        detections="$root/artifacts/evaluation/bevfusion/base3_b0_e10_class_thresholds_040_015_010_260913/obstacle_backforth/detections.bag"
        ;;
      *) echo "Unknown model: $model" >&2; exit 2 ;;
    esac
    ;;
  *) echo "Unknown scene: $scene" >&2; exit 2 ;;
esac

for path in "$raw" "$detections"; do
  [[ -f "$path" ]] || { echo "Missing required bag: $path" >&2; exit 1; }
done

port="${OBSTACLE_COMPARE_RVIZ_ROS_PORT:-$((16000 + ($$ % 12000)))}"
export ROS_MASTER_URI="http://127.0.0.1:${port}"
export ROS_IP=127.0.0.1
export ROS_HOME="/tmp/ajou_obstacle_compare_${model}_${scene}_${port}"
export AJOUNICE_ROOT="$root"
mkdir -p "$ROS_HOME"

source /opt/ros/noetic/setup.bash
source "$root/base_ws/devel/setup.bash"

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

roslaunch perception_object_pipeline tracking_prediction.launch \
  use_sim_time:=true \
  >"$ROS_HOME/tracking_prediction.log" 2>&1 &
pipeline_pid=$!

for _ in $(seq 1 300); do
  if ! kill -0 "$pipeline_pid" 2>/dev/null; then
    tail -n 100 "$ROS_HOME/tracking_prediction.log" >&2
    exit 1
  fi
  if rosnode list 2>/dev/null | grep -q '^/map_based_prediction$'; then
    break
  fi
  sleep 0.1
done

roslaunch tracking_prediction_evaluation pipeline_prediction_rviz.launch \
  use_sim_time:=true \
  start_rviz:=true \
  show_gt_boxes:=true \
  >"$ROS_HOME/rviz.log" 2>&1 &
rviz_pid=$!

for _ in $(seq 1 300); do
  if ! kill -0 "$rviz_pid" 2>/dev/null; then
    tail -n 100 "$ROS_HOME/rviz.log" >&2
    exit 1
  fi
  rosnode list 2>/dev/null | grep -q '^/object_pipeline_visualization$' && break
  sleep 0.1
done

echo "============================================================"
echo "Obstacle detector comparison: model=$model scene=$scene"
echo "Thresholds: Vehicle=0.40 Pedestrian=0.15 Obstacle=0.10"
echo "Detector -> production EKF tracking -> UNKNOWN stationary -> Map-CV"
echo "Green translucent boxes: detector output; tracked UNKNOWN is stationary"
echo "Magenta boxes: MORAI GT"
echo "SPACE: play/pause, Ctrl-C: close"
echo "============================================================"

rosbag play --clock --pause --keep-alive --rate="$rate" \
  "$raw" "$detections"
