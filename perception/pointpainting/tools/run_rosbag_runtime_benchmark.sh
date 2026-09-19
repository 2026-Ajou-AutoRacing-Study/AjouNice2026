#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 3 || $# -gt 4 ]]; then
    echo "Usage: $0 BAG_PATH PLAYBACK_RATE OUTPUT_DIR [LABEL]"
    exit 2
fi

BAG_PATH="$(realpath "$1")"
PLAYBACK_RATE="$2"
OUTPUT_DIR="$(realpath -m "$3")"
LABEL="${4:-$(basename "$BAG_PATH" .bag)_x${PLAYBACK_RATE}}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
ROS_PORT="${POINTPAINTING_BENCHMARK_ROS_PORT:-11331}"
export ROS_MASTER_URI="http://127.0.0.1:${ROS_PORT}"
export ROS_IP=127.0.0.1
export ROS_HOME="/tmp/ajou_pointpainting_benchmark_ros_${ROS_PORT}"

if [[ ! -f "$BAG_PATH" ]]; then
    echo "[ERROR] Bag not found: $BAG_PATH"
    exit 1
fi

mkdir -p "$OUTPUT_DIR" "$ROS_HOME"

ROSCORE_PID=""
NODE_PID=""

cleanup() {
    set +e
    if [[ -n "$NODE_PID" ]] && kill -0 "$NODE_PID" 2>/dev/null; then
        kill -INT "$NODE_PID" 2>/dev/null
        wait "$NODE_PID" 2>/dev/null
    fi
    if [[ -n "$ROSCORE_PID" ]] && kill -0 "$ROSCORE_PID" 2>/dev/null; then
        kill -INT "$ROSCORE_PID" 2>/dev/null
        wait "$ROSCORE_PID" 2>/dev/null
    fi
}
trap cleanup EXIT INT TERM

cd "$REPO_ROOT"
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash

roscore -p "$ROS_PORT" >"$OUTPUT_DIR/roscore.log" 2>&1 &
ROSCORE_PID=$!

for _ in $(seq 1 100); do
    if rosparam list >/dev/null 2>&1; then
        break
    fi
    sleep 0.1
done
if ! rosparam list >/dev/null 2>&1; then
    echo "[ERROR] roscore did not become ready"
    exit 1
fi

rosparam set /use_sim_time true

CONDA_ENV=openpcdet_rtx50 \
POINTPAINTING_WITH_TP=false \
POINTPAINTING_BENCHMARK_DIR="$OUTPUT_DIR" \
POINTPAINTING_BENCHMARK_LABEL="$LABEL" \
POINTPAINTING_BENCHMARK_EXCLUDE_FRAMES=10 \
POINTPAINTING_DETAILED_TIMING="${POINTPAINTING_DETAILED_TIMING:-true}" \
POINTPAINTING_PREPROCESS_BACKEND="${POINTPAINTING_PREPROCESS_BACKEND:-torchvision}" \
POINTPAINTING_CAMERA_CPU_WORKERS="${POINTPAINTING_CAMERA_CPU_WORKERS:-1}" \
base_ws/src/morai_pointpainting_ros/scripts/run_pointpainting.sh bag \
    >"$OUTPUT_DIR/pointpainting.log" 2>&1 &
NODE_PID=$!

for _ in $(seq 1 1200); do
    if ! kill -0 "$NODE_PID" 2>/dev/null; then
        echo "[ERROR] PointPainting exited during startup"
        tail -n 80 "$OUTPUT_DIR/pointpainting.log"
        exit 1
    fi
    if rostopic info /perception/object_recognition/detection/objects \
        2>/dev/null | grep -q '^Publishers:'; then
        if ! rostopic info /perception/object_recognition/detection/objects \
            2>/dev/null | grep -q '^Publishers: None'; then
            break
        fi
    fi
    sleep 0.1
done

if ! rostopic info /perception/object_recognition/detection/objects \
    2>/dev/null | grep -q '/pointpainting_online'; then
    echo "[ERROR] PointPainting publisher did not become ready"
    tail -n 80 "$OUTPUT_DIR/pointpainting.log"
    exit 1
fi

echo "[benchmark] label=$LABEL rate=$PLAYBACK_RATE bag=$BAG_PATH"
rosbag play --quiet --clock --rate="$PLAYBACK_RATE" "$BAG_PATH" \
    >"$OUTPUT_DIR/rosbag.log" 2>&1

# Let the worker publish the newest accepted LiDAR frame before shutdown.
sleep 2
kill -INT "$NODE_PID"
wait "$NODE_PID" || true
NODE_PID=""

if [[ ! -s "$OUTPUT_DIR/summary.json" ]]; then
    echo "[ERROR] summary.json was not generated"
    tail -n 120 "$OUTPUT_DIR/pointpainting.log"
    exit 1
fi

python3 - "$OUTPUT_DIR/summary.json" <<'PY'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
summary = json.loads(path.read_text(encoding="utf-8"))
rates = summary["rates_hz"]
ratios = summary["ratios"]
timing = summary["timing_ms"]
print("[benchmark] summary:", path)
print("  lidar Hz:", rates["lidar_input"])
print("  detection Hz:", rates["detection_publish"])
print("  publish/input:", ratios["publish_per_lidar_input"])
print("  drop/input:", ratios["drop_per_lidar_input"])
print("  total p50/p95 ms:", timing["total_compute_ms"]["p50"], timing["total_compute_ms"]["p95"])
print("  arrival p50/p95 ms:", timing["arrival_to_publish_ms"]["p50"], timing["arrival_to_publish_ms"]["p95"])
PY
