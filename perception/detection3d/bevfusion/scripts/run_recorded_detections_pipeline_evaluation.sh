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
tracker_detection_velocity_update_max_track_age_sec="${TRACKER_DETECTION_VELOCITY_UPDATE_MAX_TRACK_AGE_SEC:-1.0}"
tracker_center_motion_velocity_fusion="${TRACKER_CENTER_MOTION_VELOCITY_FUSION:-false}"
tracker_center_motion_fuse_ekf_state="${TRACKER_CENTER_MOTION_FUSE_EKF_STATE:-true}"
tracker_center_motion_override_output_velocity="${TRACKER_CENTER_MOTION_OVERRIDE_OUTPUT_VELOCITY:-false}"
tracker_publish_center_motion_diagnostics="${TRACKER_PUBLISH_CENTER_MOTION_DIAGNOSTICS:-false}"
tracker_center_motion_maximum_output_age_sec="${TRACKER_CENTER_MOTION_MAXIMUM_OUTPUT_AGE_SEC:-0.40}"
tracker_center_motion_min_baseline_sec="${TRACKER_CENTER_MOTION_MIN_BASELINE_SEC:-0.30}"
tracker_center_motion_medium_baseline_sec="${TRACKER_CENTER_MOTION_MEDIUM_BASELINE_SEC:-0.65}"
tracker_center_motion_long_baseline_sec="${TRACKER_CENTER_MOTION_LONG_BASELINE_SEC:-0.90}"
tracker_center_motion_max_gap_sec="${TRACKER_CENTER_MOTION_MAX_GAP_SEC:-0.25}"
tracker_center_motion_min_update_interval_sec="${TRACKER_CENTER_MOTION_MIN_UPDATE_INTERVAL_SEC:-0.20}"
tracker_center_motion_max_fit_residual_m="${TRACKER_CENTER_MOTION_MAX_FIT_RESIDUAL_M:-0.10}"
tracker_center_motion_early_noise_std_mps="${TRACKER_CENTER_MOTION_EARLY_NOISE_STD_MPS:-1.50}"
tracker_center_motion_mature_noise_std_mps="${TRACKER_CENTER_MOTION_MATURE_NOISE_STD_MPS:-0.60}"
gt_reference="${BEVFUSION_EVAL_GT_REFERENCE:-bbox-center-calibrated-v1}"
asset_bbox_catalog="${BEVFUSION_EVAL_ASSET_BBOX_CATALOG:-dataset/morai_od/configs/morai_asset_bbox_offsets_base3_v1.json}"
morai_heading_correction_deg="${BEVFUSION_EVAL_MORAI_HEADING_CORRECTION_DEG:-0.0}"
heading_correction_enabled=true
if [[ "$morai_heading_correction_deg" == "0" || "$morai_heading_correction_deg" == "0.0" ]]; then
  heading_correction_enabled=false
fi
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
if [[ "${BEVFUSION_PIPELINE_EVAL_USE_DEVEL_OVERLAY:-false}" == "true" ]]; then
  # Experimental tracker builds live in the linked devel space while the
  # competition runtime normally uses install_aw149. Keep this opt-in so an
  # offline A/B cannot silently alter the production launch contract.
  export CMAKE_PREFIX_PATH="$repo_root/base_ws/devel:${CMAKE_PREFIX_PATH:-}"
  export ROS_PACKAGE_PATH="$repo_root/base_ws/src/perception_object_pipeline:$repo_root/base_ws/src/ekf-multi-object-tracking:${ROS_PACKAGE_PATH:-}"
  export LD_LIBRARY_PATH="$repo_root/base_ws/devel/lib:${LD_LIBRARY_PATH:-}"
fi
# source_aw149 intentionally restores the production master on port 11311.
# Offline evaluation owns an isolated master, so override it afterwards.
export ROS_MASTER_URI="http://127.0.0.1:${ros_port}"
export ROS_IP=127.0.0.1
system_pythonpath="${PYTHONPATH:-$repo_root/base_ws/install_aw149/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages}"
evaluation_pythonpath="$repo_root/base_ws/src/tracking_prediction_evaluation/src:$system_pythonpath"

roscore -p "$ros_port" >"$output_dir/roscore.log" 2>&1 &
roscore_pid=$!
for _ in $(seq 1 100); do
  rosparam list >/dev/null 2>&1 && break
  sleep 0.1
done
rosparam set /use_sim_time true

env PYTHONPATH="$system_pythonpath" PYTHONNOUSERSITE=1 \
  /usr/bin/python3 /opt/ros/noetic/bin/roslaunch \
  "$repo_root/base_ws/src/bringup/launch/localization_morai_gt.launch" use_sim_time:=true \
  use_heading_correction:="$heading_correction_enabled" \
  heading_yaw_offset_deg:="$morai_heading_correction_deg" \
  >"$output_dir/localization.log" 2>&1 &
localization_pid=$!

env PYTHONPATH="$system_pythonpath" PYTHONNOUSERSITE=1 \
  /usr/bin/python3 /opt/ros/noetic/bin/roslaunch \
  "$repo_root/base_ws/src/vehicle/launch/sensor_static_tf.launch" \
  >"$output_dir/sensor_static_tf.log" 2>&1 &
sensor_tf_pid=$!

env PYTHONPATH="$system_pythonpath" PYTHONNOUSERSITE=1 \
  /usr/bin/python3 /opt/ros/noetic/bin/roslaunch \
  "$repo_root/base_ws/src/perception_object_pipeline/launch/tracking_prediction.launch" use_sim_time:=true \
  tracker_prediction_model:="$tracker_prediction_model" \
  tracker_system_noise_std_vx_vy_ms:="$tracker_system_noise_std_vx_vy_ms" \
  tracker_meas_noise_std_xy_m:="$tracker_meas_noise_std_xy_m" \
  tracker_meas_noise_std_yaw_deg:="$tracker_meas_noise_std_yaw_deg" \
  tracker_use_kinematic_model:="$tracker_use_kinematic_model" \
  tracker_detection_velocity_fusion_mode:="$tracker_detection_velocity_fusion_mode" \
  tracker_detection_velocity_noise_std_mps:="$tracker_detection_velocity_noise_std_mps" \
  tracker_detection_velocity_update_max_track_age_sec:="$tracker_detection_velocity_update_max_track_age_sec" \
  tracker_center_motion_velocity_fusion:="$tracker_center_motion_velocity_fusion" \
  tracker_center_motion_fuse_ekf_state:="$tracker_center_motion_fuse_ekf_state" \
  tracker_center_motion_override_output_velocity:="$tracker_center_motion_override_output_velocity" \
  tracker_publish_center_motion_diagnostics:="$tracker_publish_center_motion_diagnostics" \
  tracker_center_motion_maximum_output_age_sec:="$tracker_center_motion_maximum_output_age_sec" \
  tracker_center_motion_min_baseline_sec:="$tracker_center_motion_min_baseline_sec" \
  tracker_center_motion_medium_baseline_sec:="$tracker_center_motion_medium_baseline_sec" \
  tracker_center_motion_long_baseline_sec:="$tracker_center_motion_long_baseline_sec" \
  tracker_center_motion_max_gap_sec:="$tracker_center_motion_max_gap_sec" \
  tracker_center_motion_min_update_interval_sec:="$tracker_center_motion_min_update_interval_sec" \
  tracker_center_motion_max_fit_residual_m:="$tracker_center_motion_max_fit_residual_m" \
  tracker_center_motion_early_noise_std_mps:="$tracker_center_motion_early_noise_std_mps" \
  tracker_center_motion_mature_noise_std_mps:="$tracker_center_motion_mature_noise_std_mps" \
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
  /perception/object_recognition/tracking/center_motion_active_objects \
  /perception/object_recognition/tracking/center_motion_inactive_objects \
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
  "detection_velocity_noise_std_mps": $tracker_detection_velocity_noise_std_mps,
  "detection_velocity_update_max_track_age_sec": $tracker_detection_velocity_update_max_track_age_sec,
  "center_motion_velocity_fusion": $tracker_center_motion_velocity_fusion,
  "center_motion_fuse_ekf_state": $tracker_center_motion_fuse_ekf_state,
  "center_motion_override_output_velocity": $tracker_center_motion_override_output_velocity,
  "publish_center_motion_diagnostics": $tracker_publish_center_motion_diagnostics,
  "center_motion_maximum_output_age_sec": $tracker_center_motion_maximum_output_age_sec,
  "center_motion_min_baseline_sec": $tracker_center_motion_min_baseline_sec,
  "center_motion_medium_baseline_sec": $tracker_center_motion_medium_baseline_sec,
  "center_motion_long_baseline_sec": $tracker_center_motion_long_baseline_sec,
  "center_motion_max_gap_sec": $tracker_center_motion_max_gap_sec,
  "center_motion_min_update_interval_sec": $tracker_center_motion_min_update_interval_sec,
  "center_motion_max_fit_residual_m": $tracker_center_motion_max_fit_residual_m,
  "center_motion_early_noise_std_mps": $tracker_center_motion_early_noise_std_mps,
  "center_motion_mature_noise_std_mps": $tracker_center_motion_mature_noise_std_mps
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

env PYTHONPATH="$evaluation_pythonpath" PYTHONNOUSERSITE=1 /usr/bin/python3 \
  base_ws/src/tracking_prediction_evaluation/scripts/evaluate_pipeline_prediction.py \
  "$raw_bag" "$output_dir/pipeline_outputs.bag" \
  --output "$output_dir/summary.json" \
  --gt-reference "$gt_reference" \
  --asset-bbox-catalog "$asset_bbox_catalog" \
  --morai-heading-correction-deg "$morai_heading_correction_deg"

env PYTHONPATH="$evaluation_pythonpath" PYTHONNOUSERSITE=1 /usr/bin/python3 \
  base_ws/src/tracking_prediction_evaluation/scripts/evaluate_pipeline_prediction.py \
  "$raw_bag" "$output_dir/pipeline_outputs.bag" \
  --output "$output_dir/summary_cv_stable.json" \
  --gt-reference "$gt_reference" \
  --asset-bbox-catalog "$asset_bbox_catalog" \
  --morai-heading-correction-deg "$morai_heading_correction_deg" \
  --prediction-motion-slice cv-stable \
  --maximum-longitudinal-acceleration-mps2 0.5

echo "[evaluation] $label -> $output_dir/summary.json"
