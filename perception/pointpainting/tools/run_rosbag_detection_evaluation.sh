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
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BUNDLE_ROOT="${POINTPAINTING_BUNDLE_ROOT:-$HOME/Downloads/260909_pointpainting_production_3model}"
MODEL="${POINTPAINTING_EVAL_MODEL:-s1g}"
CONDA_ENV="${CONDA_ENV:-pp}"
OPENPCDET_ROOT="${OPENPCDET_ROOT:-$HOME/OpenPCDet}"
ROS_PORT="${POINTPAINTING_EVAL_ROS_PORT:-11332}"
export ROS_MASTER_URI="http://127.0.0.1:${ROS_PORT}"
export ROS_IP=127.0.0.1
export ROS_HOME="/tmp/ajou_pointpainting_eval_ros_${ROS_PORT}"

mkdir -p "$OUTPUT_DIR" "$ROS_HOME"
ROSCORE_PID=""
NODE_PID=""
RECORD_PID=""

cleanup() {
    set +e
    for pid_name in RECORD_PID NODE_PID ROSCORE_PID; do
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
source scripts/workspace/source_aw149.sh

[[ -x "$BUNDLE_ROOT/scripts/run_model.sh" ]] || {
    echo "[ERROR] PointPainting bundle entrypoint not found: $BUNDLE_ROOT/scripts/run_model.sh" >&2
    exit 1
}

roscore -p "$ROS_PORT" >"$OUTPUT_DIR/roscore.log" 2>&1 &
ROSCORE_PID=$!
for _ in $(seq 1 100); do
    rosparam list >/dev/null 2>&1 && break
    sleep 0.1
done
rosparam set /use_sim_time true

CONDA_ENV="$CONDA_ENV" \
OPENPCDET_ROOT="$OPENPCDET_ROOT" \
AJOUNICE_ROOT="$REPO_ROOT" \
POINTPAINTING_WITH_TP=false \
POINTPAINTING_PREPROCESS_BACKEND=opencv \
POINTPAINTING_CAMERA_CPU_WORKERS=4 \
POINTPAINTING_DETAILED_TIMING=true \
POINTPAINTING_BENCHMARK_DIR="$OUTPUT_DIR/runtime" \
POINTPAINTING_BENCHMARK_LABEL="$LABEL" \
POINTPAINTING_BENCHMARK_EXCLUDE_FRAMES=3 \
"$BUNDLE_ROOT/scripts/run_model.sh" "$MODEL" bag \
    >"$OUTPUT_DIR/pointpainting.log" 2>&1 &
NODE_PID=$!

for _ in $(seq 1 1200); do
    if ! kill -0 "$NODE_PID" 2>/dev/null; then
        echo "[ERROR] PointPainting exited during startup"
        tail -n 100 "$OUTPUT_DIR/pointpainting.log"
        exit 1
    fi
    if rostopic info /perception/object_recognition/detection/objects \
        2>/dev/null | grep -q '/pointpainting_online'; then
        break
    fi
    sleep 0.1
done

rosbag record --lz4 --buffsize=1024 \
    -O "$OUTPUT_DIR/detections.bag" \
    /perception/object_recognition/detection/objects \
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

/usr/bin/python3 -s \
    base_ws/src/tracking_prediction_evaluation/scripts/evaluate_rosbag_detection.py \
    "$RAW_BAG" "$OUTPUT_DIR/detections.bag" \
    --output "$OUTPUT_DIR/summary.json"

echo "[evaluation] $OUTPUT_DIR/summary.json"
