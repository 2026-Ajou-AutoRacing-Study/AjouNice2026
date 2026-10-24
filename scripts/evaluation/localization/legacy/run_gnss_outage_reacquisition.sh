#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 3 || $# -gt 4 ]]; then
  echo "usage: $0 INPUT_BAG OUTAGE_START_OFFSET_SEC OUTAGE_DURATION_SEC [LABEL]" >&2
  exit 2
fi

ROOT="${AJOUNICE_ROOT:-$HOME/AjouNice2026}"
INPUT_BAG="$(readlink -f "$1")"
OUTAGE_START_OFFSET="$2"
OUTAGE_DURATION="$3"
LABEL="${4:-outage_${OUTAGE_START_OFFSET}s_${OUTAGE_DURATION}s}"
RATE="${LOCALIZATION_OUTAGE_RATE:-1.0}"
CONTROL_TAU="${LOCALIZATION_OUTAGE_TAU_SEC:-0.5}"
CONTROL_MODE="${LOCALIZATION_OUTAGE_CONTROL_MODE:-smooth}"
GRID_YAW_DEG="${LOCALIZATION_OUTAGE_GRID_YAW_DEG:--1.347}"
PORT="${LOCALIZATION_OUTAGE_PORT:-$((26000 + ($$ % 10000)))}"
STAMP="$(date '+%y%m%d_%H%M%S')"
OUTPUT_ROOT="${LOCALIZATION_OUTAGE_OUTPUT_DIR:-$ROOT/rosbags/eval/localization/gnss_outage_$STAMP}"
RUN_DIR="$OUTPUT_ROOT/$LABEL"
FILTERED_BAG="$RUN_DIR/input_${LABEL}.bag"
OUTPUT_BAG="$RUN_DIR/localization_${LABEL}.bag"
SENSOR_CONFIG="$ROOT/configs/sensors/Sensor_competition_local.json"
ANALYZER="$ROOT/scripts/evaluation/localization/analyze_localization_stability_bag.py"
OUTAGE_ANALYZER="$ROOT/scripts/evaluation/localization/analyze_gnss_outage_reacquisition.py"

if [[ ! -f "$INPUT_BAG" ]]; then
  echo "input bag not found: $INPUT_BAG" >&2
  exit 2
fi
python3 - "$OUTAGE_START_OFFSET" "$OUTAGE_DURATION" "$CONTROL_TAU" "$GRID_YAW_DEG" <<'PY'
import math
import sys
for label, raw in zip(("start", "duration", "control tau", "grid yaw"), sys.argv[1:]):
    value = float(raw)
    if not math.isfinite(value) or (
        label != "grid yaw" and (
            value < 0.0 or (label == "control tau" and value == 0.0)
        )
    ):
        raise SystemExit("{} must be finite and {}".format(
            label,
            "any real angle" if label == "grid yaw" else (
                "positive" if label == "control tau" else "non-negative"
            ),
        ))
PY
if [[ "$CONTROL_MODE" != "smooth" && "$CONTROL_MODE" != "quantized" ]]; then
  echo "LOCALIZATION_OUTAGE_CONTROL_MODE must be smooth or quantized" >&2
  exit 2
fi

mkdir -p "$RUN_DIR"
source /opt/ros/noetic/setup.bash
source "$ROOT/scripts/workspace/source_aw149.sh"
export ROS_PACKAGE_PATH="$ROOT/base_ws/src/localization${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"
rospack profile >/dev/null

read -r BAG_START BAG_END < <(
  python3 - "$INPUT_BAG" <<'PY'
import rosbag
import sys
with rosbag.Bag(sys.argv[1], "r") as bag:
    print("{:.9f} {:.9f}".format(bag.get_start_time(), bag.get_end_time()))
PY
)
OUTAGE_START_ABS="$(python3 -c "print(float('$BAG_START') + float('$OUTAGE_START_OFFSET'))")"
OUTAGE_END_ABS="$(python3 -c "print(float('$OUTAGE_START_ABS') + float('$OUTAGE_DURATION'))")"

if ! python3 - "$BAG_START" "$BAG_END" "$OUTAGE_START_ABS" "$OUTAGE_END_ABS" <<'PY'
import sys
bag_start, bag_end, outage_start, outage_end = map(float, sys.argv[1:])
if not bag_start <= outage_start <= outage_end <= bag_end:
    raise SystemExit(
        "outage [{:.3f}, {:.3f}] is outside bag [{:.3f}, {:.3f}]".format(
            outage_start, outage_end, bag_start, bag_end
        )
    )
PY
then
  exit 2
fi

topics="['/Ego_topic','/Competition_topic','/sensor/gps','/sensor/imu','/tf_static']"
expression="(topic in $topics) and (topic != '/sensor/gps' or t.to_sec() < $OUTAGE_START_ABS or t.to_sec() > $OUTAGE_END_ABS)"
rosbag filter "$INPUT_BAG" "$FILTERED_BAG" "$expression" \
  >"$RUN_DIR/filter.log" 2>&1

printf '%s\n' \
  "input_bag=$INPUT_BAG" \
  "filtered_bag=$FILTERED_BAG" \
  "source_commit=$(git -C "$ROOT" rev-parse HEAD)" \
  "source_branch=$(git -C "$ROOT" branch --show-current)" \
  "bag_start=$BAG_START" \
  "bag_end=$BAG_END" \
  "outage_start_offset_sec=$OUTAGE_START_OFFSET" \
  "outage_duration_sec=$OUTAGE_DURATION" \
  "outage_start_absolute=$OUTAGE_START_ABS" \
  "outage_end_absolute=$OUTAGE_END_ABS" \
  "replay_rate=$RATE" \
  "heading_offset_deg=-1.347" \
  "control_translation_tau_sec=$CONTROL_TAU" \
  "control_yaw_tau_sec=$CONTROL_TAU" \
  "control_mode=$CONTROL_MODE" \
  "quantized_grid_yaw_deg=$GRID_YAW_DEG" \
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
export ROS_HOME="/tmp/ajou_localization_outage_${STAMP}_${LABEL}"
rm -rf "$ROS_HOME"
mkdir -p "$ROS_HOME"
roscore -p "$PORT" >"$RUN_DIR/roscore.log" 2>&1 &
CORE_PID=$!
for _ in $(seq 1 100); do
  rosparam list >/dev/null 2>&1 && break
  sleep 0.1
done
rosparam set /use_sim_time true

if [[ "$CONTROL_MODE" == "quantized" ]]; then
  roslaunch tf_pub map_tf.launch >"$RUN_DIR/map_tf.log" 2>&1 &
  MAP_TF_PID=$!
  roslaunch "$ROOT/base_ws/src/localization/localization_manager/launch/gnss_imu_gt_ctra.launch" \
    gt_source:=competition \
    use_heading_correction:=true \
    heading_yaw_offset_deg:=-1.347 \
    publish_control_state:=true \
    use_quantized_gnss_control_state:=true \
    quantized_control_yaw_tau_sec:=0.0 \
    quantized_control_grid_yaw_deg:="$GRID_YAW_DEG" \
    control_state_reset_translation_error_m:=0.5 \
    sensor_config:="$SENSOR_CONFIG" \
    >"$RUN_DIR/launch.log" 2>&1 &
else
  roslaunch bringup localization_gnss_imu_gt_ctra.launch \
    use_sim_time:=true \
    gt_source:=competition \
    use_heading_correction:=true \
    heading_yaw_offset_deg:=-1.347 \
    publish_sensor_tf:=true \
    publish_control_state:=true \
    control_translation_tau_sec:="$CONTROL_TAU" \
    control_yaw_tau_sec:="$CONTROL_TAU" \
    control_state_reset_translation_error_m:=0.5 \
    sensor_config:="$SENSOR_CONFIG" \
    >"$RUN_DIR/launch.log" 2>&1 &
fi
LAUNCH_PID=$!
sleep 2

rosbag record --lz4 --buffsize=256 -O "$OUTPUT_BAG" \
  /clock /Ego_topic /Competition_topic /sensor/gps /sensor/imu \
  /localization/gnss/odom /localization/odometry/local \
  /localization/odom /localization/kinematic_state \
  /localization/control_kinematic_state /tf /tf_static \
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
python3 "$OUTAGE_ANALYZER" "$OUTPUT_BAG" \
  --output "$RUN_DIR/outage_analysis.json" \
  >"$RUN_DIR/outage_analysis.stdout.json"

echo "GNSS outage replay complete: $RUN_DIR"
