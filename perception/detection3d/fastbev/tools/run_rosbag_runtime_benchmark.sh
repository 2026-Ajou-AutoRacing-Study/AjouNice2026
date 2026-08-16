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
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
ROS_PORT="${FASTBEV_BENCHMARK_ROS_PORT:-11332}"

export ROS_MASTER_URI="http://127.0.0.1:${ROS_PORT}"
export ROS_IP=127.0.0.1
export ROS_HOME="/tmp/ajou_fastbev_benchmark_ros_${ROS_PORT}"

if [[ ! -f "$BAG_PATH" ]]; then
  echo "[ERROR] Bag not found: $BAG_PATH"
  exit 1
fi

mkdir -p "$OUTPUT_DIR" "$ROS_HOME"
ROSCORE_PID=""
NODE_PID=""
MONITOR_PID=""

cleanup() {
  set +e
  for pid in "$MONITOR_PID" "$NODE_PID" "$ROSCORE_PID"; do
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

FASTBEV_WITH_TP=false \
FASTBEV_PREPROCESS_BACKEND="${FASTBEV_PREPROCESS_BACKEND:-opencv}" \
FASTBEV_PIPELINE_MODE="${FASTBEV_PIPELINE_MODE:-staged}" \
base_ws/src/morai_fastbev_ros/scripts/run_fastbev.sh bag \
  >"$OUTPUT_DIR/fastbev.log" 2>&1 &
NODE_PID=$!

for _ in $(seq 1 1200); do
  if ! kill -0 "$NODE_PID" 2>/dev/null; then
    echo "[ERROR] Fast-BEV exited during startup"
    tail -n 100 "$OUTPUT_DIR/fastbev.log"
    exit 1
  fi
  if rostopic info /fastbev/diagnostics 2>/dev/null | grep -q '/fastbev_online'; then
    break
  fi
  sleep 0.1
done

if ! rostopic info /fastbev/diagnostics 2>/dev/null | grep -q '/fastbev_online'; then
  echo "[ERROR] Fast-BEV diagnostics publisher did not become ready"
  exit 1
fi

BAG_DURATION="$(rosbag info --yaml "$BAG_PATH" | /usr/bin/python3 -c \
  'import sys, yaml; print(float(yaml.safe_load(sys.stdin)["duration"]))')"
MONITOR_DURATION="$(/usr/bin/python3 -c \
  "print(${BAG_DURATION} / float('${PLAYBACK_RATE}') + 3.0)")"

echo "[benchmark] label=$LABEL rate=$PLAYBACK_RATE bag=$BAG_PATH"
rosrun morai_fastbev_ros benchmark_fastbev_runtime.py \
  --duration "$MONITOR_DURATION" \
  --output "$OUTPUT_DIR/summary.json" \
  >"$OUTPUT_DIR/monitor.log" 2>&1 &
MONITOR_PID=$!

sleep 0.5
rosbag play --quiet --clock --rate="$PLAYBACK_RATE" "$BAG_PATH" \
  --topics \
  /sensor/camera/front/compressed \
  /sensor/camera/front_left/compressed \
  /sensor/camera/front_right/compressed \
  /sensor/camera/back/compressed \
  >"$OUTPUT_DIR/rosbag.log" 2>&1

wait "$MONITOR_PID"
MONITOR_PID=""

if [[ ! -s "$OUTPUT_DIR/summary.json" ]]; then
  echo "[ERROR] summary.json was not generated"
  tail -n 120 "$OUTPUT_DIR/monitor.log"
  exit 1
fi

python3 - "$OUTPUT_DIR/summary.json" <<'PY'
import json
import sys

d = json.load(open(sys.argv[1], encoding="utf-8"))
t = d["timings_ms"]
print("[benchmark] summary:", sys.argv[1])
print("  input Hz:", d["input"]["wall_hz"])
print("  output Hz:", d["output"]["wall_hz"])
print("  compute mean/p50/p95:", *(t["total_ms"][k] for k in ("mean", "p50", "p95")))
print("  latency mean/p50/p95:", *(t["latency_ms"][k] for k in ("mean", "p50", "p95")))
print("  steady counters:", d["counters"]["steady_state_delta"])
PY
