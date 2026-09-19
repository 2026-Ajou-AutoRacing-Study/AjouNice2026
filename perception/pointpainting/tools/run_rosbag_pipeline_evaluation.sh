#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 3 || $# -gt 4 ]]; then
    echo "Usage: $0 RAW_BAG OUTPUT_DIR LABEL [PLAYBACK_RATE]"
    exit 2
fi

RAW_BAG="$(realpath "$1")"
OUTPUT_DIR="$(realpath -m "$2")"
LABEL="$3"
PLAYBACK_RATE="${4:-1.0}"
TRACKING_INTERNAL_PUBLISH_RATE_HZ="${POINTPAINTING_TRACKING_INTERNAL_PUBLISH_RATE_HZ:-30.0}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BUNDLE_ROOT="${POINTPAINTING_BUNDLE_ROOT:-$REPO_ROOT/artifacts/team_handoffs/260909_pointpainting_production_3model}"
DETECTOR_CONDA_ENV="${CONDA_ENV:-openpcdet_rtx50}"
OPENPCDET_ROOT="${OPENPCDET_ROOT:-$HOME/OpenPCDet}"
ROS_PORT="${POINTPAINTING_PIPELINE_EVAL_ROS_PORT:-$((12000 + ($$ % 20000)))}"
export ROS_MASTER_URI="http://127.0.0.1:${ROS_PORT}"
export ROS_IP=127.0.0.1
export ROS_HOME="/tmp/ajou_pointpainting_pipeline_eval_ros_${ROS_PORT}"

mkdir -p "$OUTPUT_DIR" "$ROS_HOME"
ROSCORE_PID=""
LOCALIZATION_PID=""
SENSOR_TF_PID=""
NODE_PID=""
TRACKING_PID=""
RECORD_PID=""

cleanup() {
    set +e
    for pid_name in RECORD_PID NODE_PID TRACKING_PID SENSOR_TF_PID LOCALIZATION_PID ROSCORE_PID; do
        pid="${!pid_name}"
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            kill -INT "$pid" 2>/dev/null
            wait "$pid" 2>/dev/null
        fi
    done
}
trap cleanup EXIT INT TERM

cd "$REPO_ROOT"
export AJOUNICE_ROOT="$REPO_ROOT"
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
SYSTEM_PYTHONPATH="$REPO_ROOT/base_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages"

roscore -p "$ROS_PORT" >"$OUTPUT_DIR/roscore.log" 2>&1 &
ROSCORE_PID=$!
for _ in $(seq 1 100); do
    rosparam list >/dev/null 2>&1 && break
    sleep 0.1
done
if ! kill -0 "$ROSCORE_PID" 2>/dev/null || ! rosparam list >/dev/null 2>&1; then
    echo "[ERROR] Isolated roscore did not become ready on port $ROS_PORT"
    cat "$OUTPUT_DIR/roscore.log"
    exit 1
fi
rosparam set /use_sim_time true

# The tracker consumes lidar-frame detections but owns map-frame tracks.  Raw
# MORAI bags provide /Ego_topic rather than a pre-recorded map TF, so reproduce
# the production MORAI-GT localization branch during replay.
env PYTHONPATH="$SYSTEM_PYTHONPATH" PYTHONNOUSERSITE=1 \
/usr/bin/python3 /opt/ros/noetic/bin/roslaunch bringup localization_morai_gt.launch \
    use_sim_time:=true \
    >"$OUTPUT_DIR/localization.log" 2>&1 &
LOCALIZATION_PID=$!
env PYTHONPATH="$SYSTEM_PYTHONPATH" PYTHONNOUSERSITE=1 \
/usr/bin/python3 /opt/ros/noetic/bin/roslaunch ioniq5 sensor_static_tf.launch \
    >"$OUTPUT_DIR/sensor_static_tf.log" 2>&1 &
SENSOR_TF_PID=$!

# Keep the system-Python tracker and Map-CV process outside the detector's
# Python 3.11 Conda environment. They communicate only through ROS messages.
env PYTHONPATH="$SYSTEM_PYTHONPATH" PYTHONNOUSERSITE=1 \
/usr/bin/python3 /opt/ros/noetic/bin/roslaunch \
    perception_object_pipeline object_pipeline.launch \
    use_sim_time:=true \
    start_detector:=false \
    prediction_model:=map_based \
    force_unknown_stationary:=true \
    tracking_internal_publish_rate_hz:="$TRACKING_INTERNAL_PUBLISH_RATE_HZ" \
    repo_root:="$REPO_ROOT" \
    map_file:="$REPO_ROOT/kcity_map/lanelet_map/lanelet2_map_v84.osm" \
    projector_file:="$REPO_ROOT/kcity_map/lanelet_map/map_projector_info_v1.yaml" \
    >"$OUTPUT_DIR/tracking_map_cv.log" 2>&1 &
TRACKING_PID=$!

POINTPAINTING_DETAILED_TIMING=true \
POINTPAINTING_BENCHMARK_DIR="$OUTPUT_DIR/runtime" \
POINTPAINTING_BENCHMARK_LABEL="$LABEL" \
POINTPAINTING_BENCHMARK_EXCLUDE_FRAMES=3 \
CONDA_ENV="$DETECTOR_CONDA_ENV" \
OPENPCDET_ROOT="$OPENPCDET_ROOT" \
POINTPAINTING_RUNTIME_SETUP="$REPO_ROOT/base_ws/devel/setup.bash" \
POINTPAINTING_CFG="$BUNDLE_ROOT/models/b3s_pp1_e34/model.yaml" \
POINTPAINTING_CKPT="$BUNDLE_ROOT/models/b3s_pp1_e34/detector.pth" \
POINTPAINTING_SEG_CKPT="$BUNDLE_ROOT/segmentation/base3_lraspp_e30/segmentation.pth" \
POINTPAINTING_WITH_TP=false \
POINTPAINTING_SCORE_THRESHOLD=0.10 \
POINTPAINTING_MODEL_SCORE_THRESHOLD=0.10 \
POINTPAINTING_VEHICLE_SCORE_THRESHOLD=0.40 \
POINTPAINTING_PEDESTRIAN_SCORE_THRESHOLD=0.15 \
POINTPAINTING_OBSTACLE_SCORE_THRESHOLD=0.10 \
POINTPAINTING_PUBLISH_DETECTOR_VELOCITY=false \
POINTPAINTING_PREPROCESS_BACKEND=opencv \
POINTPAINTING_CAMERA_CPU_WORKERS=4 \
POINTPAINTING_LIDAR_MEASUREMENT_STAMP_MODE=object_points_median \
POINTPAINTING_LIDAR_DESKEW_MODE=off \
POINTPAINTING_TEMPORAL_NUM_SWEEPS=0 \
base_ws/src/morai_pointpainting_ros/scripts/run_pointpainting.sh bag \
    >"$OUTPUT_DIR/pointpainting_pipeline.log" 2>&1 &
NODE_PID=$!

pipeline_ready=false
for _ in $(seq 1 1200); do
    if ! kill -0 "$NODE_PID" 2>/dev/null; then
        echo "[ERROR] PointPainting detector exited during startup"
        tail -n 150 "$OUTPUT_DIR/pointpainting_pipeline.log"
        exit 1
    fi
    if ! kill -0 "$TRACKING_PID" 2>/dev/null; then
        echo "[ERROR] Tracking/Map-CV pipeline exited during startup"
        tail -n 150 "$OUTPUT_DIR/tracking_map_cv.log"
        exit 1
    fi
    detection_info="$(rostopic info /perception/object_recognition/detection/objects 2>/dev/null || true)"
    measurement_info="$(rostopic info /perception/object_recognition/tracking/measurement_objects 2>/dev/null || true)"
    tracking_info="$(rostopic info /perception/object_recognition/tracking/objects 2>/dev/null || true)"
    prediction_info="$(rostopic info /perception/object_recognition/objects 2>/dev/null || true)"
    if grep -q '/pointpainting_online' <<<"$detection_info" && \
       grep -q '/ekf_multi_object_tracking' <<<"$detection_info" && \
       grep -q '/ekf_multi_object_tracking' <<<"$measurement_info" && \
       grep -q '/tracked_object_motion_policy' <<<"$measurement_info" && \
       grep -q '/tracked_object_motion_policy' <<<"$tracking_info" && \
       grep -q '/map_based_prediction' <<<"$tracking_info" && \
       grep -q '/map_based_prediction' <<<"$prediction_info"; then
        pipeline_ready=true
        break
    fi
    sleep 0.1
done
if [[ "$pipeline_ready" != true ]]; then
    echo "[ERROR] Detector -> EKF -> motion policy -> Map-CV graph did not become ready"
    tail -n 150 "$OUTPUT_DIR/pointpainting_pipeline.log"
    tail -n 150 "$OUTPUT_DIR/tracking_map_cv.log"
    exit 1
fi

# Every launch participating in an offline run must agree on bag time.  A
# launch file with a default use_sim_time:=false can otherwise silently reset
# the global parameter after roscore startup, making the EKF compare wall time
# against recorded sensor timestamps and discard every fixed-rate output.
if [[ "$(rosparam get /use_sim_time 2>/dev/null || true)" != "true" ]]; then
    echo "[ERROR] /use_sim_time was reset while starting the offline pipeline" >&2
    echo "All included launch files must receive use_sim_time:=true." >&2
    exit 1
fi

rosbag record --lz4 --buffsize=1024 \
    -O "$OUTPUT_DIR/pipeline_outputs.bag" \
    /perception/object_recognition/detection/objects \
    /perception/object_recognition/tracking/objects \
    /perception/object_recognition/objects \
    /tf /tf_static \
    >"$OUTPUT_DIR/record.log" 2>&1 &
RECORD_PID=$!
sleep 0.5

rosbag play --quiet --clock --rate="$PLAYBACK_RATE" "$RAW_BAG" \
    >"$OUTPUT_DIR/rosbag.log" 2>&1
sleep 2

kill -INT "$RECORD_PID" 2>/dev/null || true
wait "$RECORD_PID" || true
RECORD_PID=""
kill -INT "$NODE_PID" 2>/dev/null || true
wait "$NODE_PID" || true
NODE_PID=""
kill -INT "$TRACKING_PID" 2>/dev/null || true
wait "$TRACKING_PID" || true
TRACKING_PID=""
kill -INT "$LOCALIZATION_PID" 2>/dev/null || true
wait "$LOCALIZATION_PID" || true
LOCALIZATION_PID=""
kill -INT "$SENSOR_TF_PID" 2>/dev/null || true
wait "$SENSOR_TF_PID" || true
SENSOR_TF_PID=""

/usr/bin/python3 \
    base_ws/src/tracking_prediction_evaluation/scripts/evaluate_pipeline_prediction.py \
    "$RAW_BAG" "$OUTPUT_DIR/pipeline_outputs.bag" \
    --output "$OUTPUT_DIR/summary.json"

/usr/bin/python3 \
    base_ws/src/tracking_prediction_evaluation/scripts/evaluate_pipeline_prediction.py \
    "$RAW_BAG" "$OUTPUT_DIR/pipeline_outputs.bag" \
    --output "$OUTPUT_DIR/summary_cv_stable.json" \
    --prediction-motion-slice cv-stable \
    --maximum-longitudinal-acceleration-mps2 0.5

echo "[evaluation] $OUTPUT_DIR/summary.json"
echo "[evaluation] $OUTPUT_DIR/summary_cv_stable.json"
