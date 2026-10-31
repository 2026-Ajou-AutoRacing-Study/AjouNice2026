#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 INPUT_BAG" >&2
  exit 2
fi

ROOT="${AJOUNICE_ROOT:-$HOME/AjouNice2026}"
INPUT_BAG="$(readlink -f "$1")"
RATE="${LOCALIZATION_YAW_RATE_AB_RATE:-1.0}"
PORT_BASE="${LOCALIZATION_YAW_RATE_AB_PORT_BASE:-$((32000 + ($$ % 6000)))}"
STAMP="$(date '+%y%m%d_%H%M%S')"
OUTPUT_ROOT="${LOCALIZATION_YAW_RATE_AB_OUTPUT_DIR:-$ROOT/rosbags/eval/localization/competition_yaw_rate_ab_$STAMP}"
SENSOR_CONFIG="$ROOT/configs/sensors/Sensor_competition_local.json"
ANALYZER="$ROOT/scripts/evaluation/localization/analyze_localization_stability_bag.py"

if [[ ! -f "$INPUT_BAG" ]]; then
  echo "input bag not found: $INPUT_BAG" >&2
  exit 2
fi
if ! rosbag info --yaml "$INPUT_BAG" | grep -q '/competition/raw_twist'; then
  echo "input bag has no /competition/raw_twist" >&2
  exit 2
fi

mkdir -p "$OUTPUT_ROOT"
source /opt/ros/noetic/setup.bash
source "$ROOT/scripts/workspace/source_aw149.sh"

CORE_PID=""; ADAPTER_PID=""; LAUNCH_PID=""; RECORD_PID=""

stop_process() {
  local pid="$1"
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    kill -INT "$pid" 2>/dev/null || true
    for _ in $(seq 1 50); do kill -0 "$pid" 2>/dev/null || break; sleep 0.1; done
    kill -0 "$pid" 2>/dev/null && kill -TERM "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  fi
}

cleanup_run() {
  stop_process "$RECORD_PID"; stop_process "$LAUNCH_PID"
  stop_process "$ADAPTER_PID"; stop_process "$CORE_PID"
  RECORD_PID=""; LAUNCH_PID=""; ADAPTER_PID=""; CORE_PID=""
}
trap cleanup_run EXIT INT TERM

if [[ -n "${LOCALIZATION_YAW_RATE_AB_PROFILES:-}" ]]; then
  read -r -a profiles <<<"$LOCALIZATION_YAW_RATE_AB_PROFILES"
else
  profiles=("imu:/sensor/imu" "competition:/localization/competition_imu")
fi
for index in "${!profiles[@]}"; do
  IFS=: read -r label imu_topic <<<"${profiles[$index]}"
  run_dir="$OUTPUT_ROOT/$label"; mkdir -p "$run_dir"
  output_bag="$run_dir/localization_${label}.bag"
  port=$((PORT_BASE + index))
  export ROS_MASTER_URI="http://127.0.0.1:$port"
  export ROS_HOME="/tmp/ajou_localization_yaw_rate_${STAMP}_${label}"
  rm -rf "$ROS_HOME" && mkdir -p "$ROS_HOME"

  printf '%s\n' \
    "input_bag=$INPUT_BAG" \
    "source_commit=$(git -C "$ROOT" rev-parse HEAD)" \
    "profile=$label" "imu_topic=$imu_topic" \
    "competition_yaw_rate_scale=-pi/180" "replay_rate=$RATE" \
    >"$run_dir/metadata.txt"

  roscore -p "$port" >"$run_dir/roscore.log" 2>&1 & CORE_PID=$!
  for _ in $(seq 1 100); do rosparam list >/dev/null 2>&1 && break; sleep 0.1; done
  rosparam set /use_sim_time true

  if [[ "$label" == "competition" ]]; then
    python3 "$ROOT/base_ws/src/localization/localization_manager/scripts/competition_yaw_rate_imu_adapter.py" \
      >"$run_dir/adapter.log" 2>&1 & ADAPTER_PID=$!
    sleep 0.5
  fi

  roslaunch bringup localization_gnss_imu_gt_ctra.launch \
    use_sim_time:=true gt_source:=competition \
    use_heading_correction:=true heading_yaw_offset_deg:=-1.347 \
    publish_sensor_tf:=true sensor_config:="$SENSOR_CONFIG" \
    imu_topic:="$imu_topic" publish_control_state:=false \
    >"$run_dir/launch.log" 2>&1 & LAUNCH_PID=$!
  sleep 2

  rosbag record --lz4 --buffsize=256 -O "$output_bag" \
    /clock /Ego_topic /Competition_topic /competition/raw_twist /sensor/imu \
    /localization/competition_imu /sensor/gps /localization/gnss/odom \
    /localization/odometry/local /localization/odom \
    /localization/kinematic_state /tf /tf_static \
    >"$run_dir/record.log" 2>&1 & RECORD_PID=$!
  sleep 1

  echo "[$label] localization IMU source=$imu_topic"
  rosbag play --clock --rate "$RATE" "$INPUT_BAG" \
    --topics /Ego_topic /Competition_topic /competition/raw_twist \
    /sensor/gps /sensor/imu /tf_static >"$run_dir/play.log" 2>&1
  sleep 1
  cleanup_run
  if [[ "$label" == "competition" ]]; then
    python3 - "$output_bag" <<'PY'
import rosbag
import sys

with rosbag.Bag(sys.argv[1]) as bag:
    count = bag.get_message_count("/localization/competition_imu")
if count < 100:
    raise SystemExit(
        "invalid Competition yaw-rate run: only {} adapter messages".format(count)
    )
print("Competition adapter messages: {}".format(count))
PY
  fi
  python3 "$ANALYZER" "$output_bag" --output "$run_dir/analysis.json" \
    >"$run_dir/analysis.stdout.json"
done

echo "Competition yaw-rate A/B complete: $OUTPUT_ROOT"
