#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 5 || $# -gt 5 ]]; then
  echo "Usage: $0 RAW_BAG DETECTIONS_BAG OUTPUT_DIR LABEL PLAYBACK_RATE" >&2
  exit 2
fi

raw_bag="$(realpath "$1")"
detections_bag="$(realpath "$2")"
output_dir="$(realpath -m "$3")"
label="$4"
playback_rate="$5"
tracker_prediction_model="${TRACKER_PREDICTION_MODEL:-1}"
tracker_system_noise_std_vx_vy_ms="${TRACKER_SYSTEM_NOISE_STD_VX_VY_MS:-1.0}"
tracker_meas_noise_std_xy_m="${TRACKER_MEAS_NOISE_STD_XY_M:-0.4}"
tracker_meas_noise_std_yaw_deg="${TRACKER_MEAS_NOISE_STD_YAW_DEG:-10.0}"
tracker_use_kinematic_model="${TRACKER_USE_KINEMATIC_MODEL:-false}"
tracker_detection_velocity_fusion_mode="${TRACKER_DETECTION_VELOCITY_FUSION_MODE:-0}"
tracker_detection_velocity_noise_std_mps="${TRACKER_DETECTION_VELOCITY_NOISE_STD_MPS:-1.0}"
repo_root="${AJOUNICE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"
ros_port="${BEVFUSION_PIPELINE_EVAL_ROS_PORT:-$((15000 + ($$ % 15000)))}"
export ROS_MASTER_URI="http://127.0.0.1:${ros_port}"
export ROS_IP=127.0.0.1
export ROS_HOME="/tmp/ajou_bevfusion_pipeline_eval_${ros_port}"

mkdir -p "$output_dir" "$ROS_HOME"
roscore_pid=""
localization_pid=""
sensor_tf_pid=""
pipeline_pid=""
record_pid=""

cleanup() {
  set +e
  for pid in "$record_pid" "$pipeline_pid" "$sensor_tf_pid" \
      "$localization_pid" "$roscore_pid"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null
      wait "$pid" 2>/dev/null
    fi
  done
}
trap cleanup EXIT INT TERM

cd "$repo_root"
export AJOUNICE_ROOT="$repo_root"
source /opt/ros/noetic/setup.bash
source scripts/workspace/source_aw149.sh
# source_aw149 intentionally restores the production master on port 11311.
# Offline evaluation owns an isolated master, so override it afterwards.
export ROS_MASTER_URI="http://127.0.0.1:${ros_port}"
export ROS_IP=127.0.0.1
system_pythonpath="${PYTHONPATH:-$repo_root/base_ws/install_aw149/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages}"

roscore -p "$ros_port" >"$output_dir/roscore.log" 2>&1 &
roscore_pid=$!
for _ in $(seq 1 100); do
  rosparam list >/dev/null 2>&1 && break
  sleep 0.1
done
rosparam set /use_sim_time true

env PYTHONPATH="$system_pythonpath" PYTHONNOUSERSITE=1 \
  /usr/bin/python3 /opt/ros/noetic/bin/roslaunch \
  bringup localization_morai_gt.launch use_sim_time:=true \
  >"$output_dir/localization.log" 2>&1 &
localization_pid=$!

env PYTHONPATH="$system_pythonpath" PYTHONNOUSERSITE=1 \
  /usr/bin/python3 /opt/ros/noetic/bin/roslaunch \
  ioniq5 sensor_static_tf.launch \
  >"$output_dir/sensor_static_tf.log" 2>&1 &
sensor_tf_pid=$!

env PYTHONPATH="$system_pythonpath" PYTHONNOUSERSITE=1 \
  /usr/bin/python3 /opt/ros/noetic/bin/roslaunch \
  perception_object_pipeline tracking_prediction.launch use_sim_time:=true \
  tracker_prediction_model:="$tracker_prediction_model" \
  tracker_system_noise_std_vx_vy_ms:="$tracker_system_noise_std_vx_vy_ms" \
  tracker_meas_noise_std_xy_m:="$tracker_meas_noise_std_xy_m" \
  tracker_meas_noise_std_yaw_deg:="$tracker_meas_noise_std_yaw_deg" \
  tracker_use_kinematic_model:="$tracker_use_kinematic_model" \
  tracker_detection_velocity_fusion_mode:="$tracker_detection_velocity_fusion_mode" \
  tracker_detection_velocity_noise_std_mps:="$tracker_detection_velocity_noise_std_mps" \
  >"$output_dir/tracking_map_cv.log" 2>&1 &
pipeline_pid=$!

ready=false
for _ in $(seq 1 600); do
  if ! kill -0 "$pipeline_pid" 2>/dev/null; then
    echo "[ERROR] Tracking/Map-CV pipeline exited during startup" >&2
    tail -n 120 "$output_dir/tracking_map_cv.log" >&2
    exit 1
  fi
  detection_info="$(rostopic info /perception/object_recognition/detection/objects 2>/dev/null || true)"
  tracking_info="$(rostopic info /perception/object_recognition/tracking/objects 2>/dev/null || true)"
  prediction_info="$(rostopic info /perception/object_recognition/objects 2>/dev/null || true)"
  if grep -q '/ekf_multi_object_tracking' <<<"$detection_info" && \
      grep -q '/map_based_prediction' <<<"$tracking_info" && \
      grep -q '/map_based_prediction' <<<"$prediction_info"; then
    ready=true
    break
  fi
  sleep 0.1
done
if [[ "$ready" != true ]]; then
  echo "[ERROR] Detection -> tracking -> Map-CV graph did not become ready" >&2
  exit 1
fi
if [[ "$(rosparam get /use_sim_time 2>/dev/null || true)" != true ]]; then
  echo "[ERROR] /use_sim_time was reset during pipeline startup" >&2
  exit 1
fi

rosbag record --lz4 --buffsize=1024 \
  -O "$output_dir/pipeline_outputs.bag" \
  /perception/object_recognition/detection/objects \
  /perception/object_recognition/tracking/objects \
  /perception/object_recognition/objects \
  /tf /tf_static \
  >"$output_dir/record.log" 2>&1 &
record_pid=$!
sleep 0.5

cat >"$output_dir/tracker_config.json" <<EOF
{
  "prediction_model": $tracker_prediction_model,
  "system_noise_std_vx_vy_ms": $tracker_system_noise_std_vx_vy_ms,
  "meas_noise_std_xy_m": $tracker_meas_noise_std_xy_m,
  "meas_noise_std_yaw_deg": $tracker_meas_noise_std_yaw_deg,
  "use_kinematic_model": $tracker_use_kinematic_model,
  "detection_velocity_fusion_mode": $tracker_detection_velocity_fusion_mode,
  "detection_velocity_noise_std_mps": $tracker_detection_velocity_noise_std_mps
}
EOF

echo "[play] $label rate=$playback_rate tracker=$(tr -d '\n ' < "$output_dir/tracker_config.json")"
# The pipeline needs only ego pose and recorded detections. Re-publishing the
# large camera/LiDAR payloads from the raw bag can starve the tracker's 30 Hz
# ROS-time scheduler and make identical runs produce different frame counts.
rosbag play --quiet --clock --rate="$playback_rate" \
  "$raw_bag" "$detections_bag" \
  --topics /Ego_topic /perception/object_recognition/detection/objects \
  >"$output_dir/rosbag.log" 2>&1
sleep 2

kill -INT "$record_pid" 2>/dev/null || true
wait "$record_pid" || true
record_pid=""
kill -INT "$pipeline_pid" 2>/dev/null || true
wait "$pipeline_pid" || true
pipeline_pid=""

/usr/bin/python3 \
  base_ws/src/tracking_prediction_evaluation/scripts/evaluate_pipeline_prediction.py \
  "$raw_bag" "$output_dir/pipeline_outputs.bag" \
  --output "$output_dir/summary.json"

/usr/bin/python3 \
  base_ws/src/tracking_prediction_evaluation/scripts/evaluate_pipeline_prediction.py \
  "$raw_bag" "$output_dir/pipeline_outputs.bag" \
  --output "$output_dir/summary_cv_stable.json" \
  --prediction-motion-slice cv-stable \
  --maximum-longitudinal-acceleration-mps2 0.5

echo "[evaluation] $label -> $output_dir/summary.json"
