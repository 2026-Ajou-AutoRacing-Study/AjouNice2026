#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "usage: $0 INPUT_BAG [LABEL]" >&2
  exit 2
fi

ROOT="${AJOUNICE_ROOT:-$HOME/AjouNice2026}"
INPUT_BAG="$(readlink -f "$1")"
LABEL="${2:-checkpoint_relocation_20m}"
PORT="${LOCALIZATION_RELOCATION_PORT:-$((27000 + ($$ % 10000)))}"
RATE="${LOCALIZATION_RELOCATION_RATE:-1.0}"
DURATION="${LOCALIZATION_RELOCATION_DURATION_SEC:-25.0}"
OFFSET_START="${LOCALIZATION_RELOCATION_OFFSET_START_SEC:-12.5}"
NORTH_OFFSET_M="${LOCALIZATION_RELOCATION_NORTH_OFFSET_M:-20.0}"
STAMP="$(date '+%y%m%d_%H%M%S')"
OUTPUT_ROOT="${LOCALIZATION_RELOCATION_OUTPUT_DIR:-$ROOT/rosbags/eval/localization/checkpoint_relocation_$STAMP}"
RUN_DIR="$OUTPUT_ROOT/$LABEL"
FILTERED_BAG="$RUN_DIR/input_${LABEL}.bag"
OUTPUT_BAG="$RUN_DIR/localization_${LABEL}.bag"
SENSOR_CONFIG="$ROOT/configs/sensors/Sensor_competition_local.json"
ANALYZER="$ROOT/scripts/evaluation/localization/analyze_checkpoint_relocation_replay.py"

if [[ ! -f "$INPUT_BAG" ]]; then
  echo "input bag not found: $INPUT_BAG" >&2
  exit 2
fi
python3 - "$DURATION" "$OFFSET_START" "$NORTH_OFFSET_M" <<'PY'
import math
import sys
duration, start, offset = map(float, sys.argv[1:])
if not all(math.isfinite(value) for value in (duration, start, offset)):
    raise SystemExit("duration, offset start, and north offset must be finite")
if duration <= 0.0 or start <= 0.0 or start >= duration or abs(offset) < 5.0:
    raise SystemExit("require duration > offset start > 0 and |north offset| >= 5m")
PY

mkdir -p "$RUN_DIR"
source /opt/ros/noetic/setup.bash
source "$ROOT/scripts/workspace/source_aw149.sh"
export ROS_PACKAGE_PATH="$ROOT/base_ws/src/localization${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"
rospack profile >/dev/null

BAG_START="$(python3 - "$INPUT_BAG" <<'PY'
import rosbag
import sys
with rosbag.Bag(sys.argv[1], "r") as bag:
    print("{:.9f}".format(bag.get_start_time()))
PY
)"
BAG_END_ABS="$(python3 -c "print(float('$BAG_START') + float('$DURATION'))")"
OFFSET_START_ABS="$(python3 -c "print(float('$BAG_START') + float('$OFFSET_START'))")"
# Latitude-only injection is sufficient for a deterministic northward checkpoint
# relocation at K-City.  The WGS84 meridional scale error is far below the 5m guard.
LATITUDE_OFFSET_DEG="$(python3 -c "print(float('$NORTH_OFFSET_M') / 111320.0)")"
topics="['/Ego_topic','/Competition_topic','/sensor/gps','/sensor/imu','/tf_static']"
expression="(topic in $topics) and (t.to_sec() <= $BAG_END_ABS) and (topic != '/sensor/gps' or t.to_sec() < $OFFSET_START_ABS or (setattr(m, 'latitude', m.latitude + $LATITUDE_OFFSET_DEG) is None))"
rosbag filter "$INPUT_BAG" "$FILTERED_BAG" "$expression" \
  >"$RUN_DIR/filter.log" 2>&1

printf '%s\n' \
  "input_bag=$INPUT_BAG" \
  "filtered_bag=$FILTERED_BAG" \
  "source_commit=$(git -C "$ROOT" rev-parse HEAD)" \
  "source_branch=$(git -C "$ROOT" branch --show-current)" \
  "duration_sec=$DURATION" \
  "offset_start_sec=$OFFSET_START" \
  "north_offset_m=$NORTH_OFFSET_M" \
  "latitude_offset_deg=$LATITUDE_OFFSET_DEG" \
  "replay_rate=$RATE" \
  "heading_offset_deg=-1.347" \
  "control_mode=quantized" \
  "relocation_guard=active" \
  >"$RUN_DIR/metadata.txt"

CORE_PID=""
MAP_TF_PID=""
LAUNCH_PID=""
RECORD_PID=""
cleanup_process() {
  local pid="$1"
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    kill -INT "$pid" 2>/dev/null || true
    for _ in $(seq 1 50); do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.1
    done
    kill -0 "$pid" 2>/dev/null && kill -TERM "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  fi
}
cleanup() {
  cleanup_process "$RECORD_PID"
  cleanup_process "$LAUNCH_PID"
  cleanup_process "$MAP_TF_PID"
  cleanup_process "$CORE_PID"
}
trap cleanup EXIT INT TERM

export ROS_MASTER_URI="http://127.0.0.1:$PORT"
export ROS_HOME="/tmp/ajou_localization_relocation_${STAMP}_${LABEL}"
rm -rf "$ROS_HOME"
mkdir -p "$ROS_HOME"
roscore -p "$PORT" >"$RUN_DIR/roscore.log" 2>&1 &
CORE_PID=$!
for _ in $(seq 1 100); do
  rosparam list >/dev/null 2>&1 && break
  sleep 0.1
done
rosparam set /use_sim_time true

roslaunch tf_pub map_tf.launch >"$RUN_DIR/map_tf.log" 2>&1 &
MAP_TF_PID=$!
roslaunch "$ROOT/base_ws/src/localization/localization_manager/launch/gnss_imu_gt_ctra.launch" \
  gt_source:=competition \
  use_heading_correction:=true \
  heading_yaw_offset_deg:=-1.347 \
  publish_control_state:=true \
  use_quantized_gnss_control_state:=true \
  quantized_control_yaw_tau_sec:=0.0 \
  control_state_reset_translation_error_m:=0.5 \
  enable_relocation_guard:=true \
  relocation_guard_monitor_only:=false \
  sensor_config:="$SENSOR_CONFIG" \
  >"$RUN_DIR/launch.log" 2>&1 &
LAUNCH_PID=$!
sleep 2

rosbag record --lz4 --buffsize=256 -O "$OUTPUT_BAG" \
  /clock /Ego_topic /Competition_topic /sensor/gps /sensor/imu \
  /localization/initialpose /localization/gnss/odom \
  /localization/odometry/local /localization/odom \
  /localization/kinematic_state /localization/control_kinematic_state \
  /tf /tf_static \
  >"$RUN_DIR/record.log" 2>&1 &
RECORD_PID=$!
sleep 1

rosbag play --clock --rate "$RATE" "$FILTERED_BAG" \
  --topics /Ego_topic /Competition_topic /sensor/gps /sensor/imu /tf_static \
  >"$RUN_DIR/play.log" 2>&1
sleep 1
cleanup
CORE_PID=""
MAP_TF_PID=""
LAUNCH_PID=""
RECORD_PID=""

python3 "$ANALYZER" "$OUTPUT_BAG" \
  --output "$RUN_DIR/analysis.json" \
  >"$RUN_DIR/analysis.stdout.json"

echo "Checkpoint relocation replay complete: $RUN_DIR"
