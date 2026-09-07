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
SCORE_THRESHOLD="${POINTPAINTING_SCORE_THRESHOLD:-0.10}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
ROS_PORT="${POINTPAINTING_PIPELINE_EVAL_ROS_PORT:-11333}"
export ROS_MASTER_URI="http://127.0.0.1:${ROS_PORT}"
export ROS_IP=127.0.0.1
export ROS_HOME="/tmp/ajou_pointpainting_pipeline_eval_ros_${ROS_PORT}"

mkdir -p "$OUTPUT_DIR" "$ROS_HOME"
ROSCORE_PID=""
LOCALIZATION_PID=""
SENSOR_TF_PID=""
NODE_PID=""
RECORD_PID=""

cleanup() {
    set +e
    for pid_name in RECORD_PID NODE_PID SENSOR_TF_PID LOCALIZATION_PID ROSCORE_PID; do
        pid="${!pid_name}"
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            kill -INT "$pid" 2>/dev/null
            wait "$pid" 2>/dev/null
        fi
    done
}
trap cleanup EXIT INT TERM

cd "$REPO_ROOT"
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash

roscore -p "$ROS_PORT" >"$OUTPUT_DIR/roscore.log" 2>&1 &
ROSCORE_PID=$!
for _ in $(seq 1 100); do
    rosparam list >/dev/null 2>&1 && break
    sleep 0.1
done
rosparam set /use_sim_time true

# The tracker consumes lidar-frame detections but owns map-frame tracks.  Raw
# MORAI bags provide /Ego_topic rather than a pre-recorded map TF, so reproduce
# the production MORAI-GT localization branch during replay.
roslaunch bringup localization_morai_gt.launch \
    >"$OUTPUT_DIR/localization.log" 2>&1 &
LOCALIZATION_PID=$!
roslaunch ioniq5 sensor_static_tf.launch \
    >"$OUTPUT_DIR/sensor_static_tf.log" 2>&1 &
SENSOR_TF_PID=$!

CONDA_ENV=openpcdet_rtx50 \
POINTPAINTING_WITH_TP=true \
POINTPAINTING_PREDICTION_MODEL=map_based \
POINTPAINTING_SCORE_THRESHOLD="$SCORE_THRESHOLD" \
POINTPAINTING_PREPROCESS_BACKEND=opencv \
POINTPAINTING_CAMERA_CPU_WORKERS=4 \
POINTPAINTING_DETAILED_TIMING=true \
POINTPAINTING_BENCHMARK_DIR="$OUTPUT_DIR/runtime" \
POINTPAINTING_BENCHMARK_LABEL="$LABEL" \
POINTPAINTING_BENCHMARK_EXCLUDE_FRAMES=3 \
base_ws/src/morai_pointpainting_ros/scripts/run_pointpainting.sh bag \
    >"$OUTPUT_DIR/pointpainting_pipeline.log" 2>&1 &
NODE_PID=$!

for _ in $(seq 1 1200); do
    if ! kill -0 "$NODE_PID" 2>/dev/null; then
        echo "[ERROR] PointPainting pipeline exited during startup"
        tail -n 150 "$OUTPUT_DIR/pointpainting_pipeline.log"
        exit 1
    fi
    if rostopic info /perception/object_recognition/objects \
        2>/dev/null | grep -q '/map_based_prediction'; then
        break
    fi
    sleep 0.1
done

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

kill -INT "$RECORD_PID"
wait "$RECORD_PID" || true
RECORD_PID=""
kill -INT "$NODE_PID"
wait "$NODE_PID" || true
NODE_PID=""
kill -INT "$LOCALIZATION_PID"
wait "$LOCALIZATION_PID" || true
LOCALIZATION_PID=""
kill -INT "$SENSOR_TF_PID"
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
