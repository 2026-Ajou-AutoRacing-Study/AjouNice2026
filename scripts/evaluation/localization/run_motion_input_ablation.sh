#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 INPUT_BAG" >&2
  exit 2
fi

ROOT="${AJOUNICE_ROOT:-$HOME/AjouNice2026}"
INPUT_BAG="$(readlink -f "$1")"
RATE="${LOCALIZATION_ABLATION_RATE:-3.0}"
PORT_BASE="${LOCALIZATION_ABLATION_PORT_BASE:-$((24000 + ($$ % 10000)))}"
STAMP="$(date '+%y%m%d_%H%M%S')"
OUTPUT_ROOT="${LOCALIZATION_ABLATION_OUTPUT_DIR:-$ROOT/rosbags/eval/localization/motion_input_ablation_$STAMP}"
SOURCE_OVERRIDE="$ROOT/configs/localization/robot_localization_gnss_imu_gt_ctra_local.yaml"
SENSOR_CONFIG="$ROOT/configs/sensors/Sensor_competition_local.json"
ANALYZER="$ROOT/scripts/evaluation/localization/analyze_localization_stability_bag.py"

if [[ ! -f "$INPUT_BAG" ]]; then
  echo "input bag not found: $INPUT_BAG" >&2
  exit 2
fi

mkdir -p "$OUTPUT_ROOT"
source /opt/ros/noetic/setup.bash
source "$ROOT/scripts/workspace/source_aw149.sh"

CORE_PID=""
LAUNCH_PID=""
RECORD_PID=""

stop_process() {
  local pid="$1"
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    kill -INT "$pid" 2>/dev/null || true
    for _ in $(seq 1 50); do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
    wait "$pid" 2>/dev/null || true
  fi
}

cleanup_run() {
  stop_process "$RECORD_PID"
  stop_process "$LAUNCH_PID"
  stop_process "$CORE_PID"
  RECORD_PID=""
  LAUNCH_PID=""
  CORE_PID=""
}
trap cleanup_run EXIT INT TERM

write_override() {
  local destination="$1"
  local use_yaw_rate="$2"
  local use_acceleration="$3"
  python3 - "$SOURCE_OVERRIDE" "$destination" "$use_yaw_rate" "$use_acceleration" <<'PY'
import sys
import yaml

source, destination, use_yaw_rate, use_acceleration = sys.argv[1:]
with open(source) as stream:
    config = yaml.safe_load(stream)
imu = list(config["imu0_config"])
imu[11] = use_yaw_rate == "true"
imu[12] = use_acceleration == "true"
config["imu0_config"] = imu
with open(destination, "w") as stream:
    yaml.safe_dump(config, stream, default_flow_style=False, sort_keys=False)
PY
}

if [[ -n "${LOCALIZATION_ABLATION_PROFILES:-}" ]]; then
  read -r -a profiles <<<"$LOCALIZATION_ABLATION_PROFILES"
else
  profiles=(
    "baseline:true:true"
    "no_ax:true:false"
    "no_yaw_rate:false:true"
    "heading_vx_only:false:false"
  )
fi

for index in "${!profiles[@]}"; do
  IFS=: read -r label use_yaw_rate use_acceleration <<<"${profiles[$index]}"
  run_dir="$OUTPUT_ROOT/$label"
  mkdir -p "$run_dir"
  local_override="$run_dir/local_override.yaml"
  global_override="$run_dir/global_override.yaml"
  output_bag="$run_dir/localization_${label}.bag"
  port=$((PORT_BASE + index))
  ros_home="/tmp/ajou_localization_ablation_${STAMP}_${label}"
  rm -rf "$ros_home"
  mkdir -p "$ros_home"

  write_override "$local_override" "$use_yaw_rate" "$use_acceleration"
  # The local/global GT override contract is identical except for
  # publish_acceleration, which is retained from the tracked global file.
  write_override "$global_override" "$use_yaw_rate" "$use_acceleration"
  python3 - "$global_override" <<'PY'
import sys
import yaml
path = sys.argv[1]
with open(path) as stream:
    config = yaml.safe_load(stream)
config["publish_acceleration"] = True
with open(path, "w") as stream:
    yaml.safe_dump(config, stream, default_flow_style=False, sort_keys=False)
PY

  export ROS_MASTER_URI="http://127.0.0.1:$port"
  export ROS_HOME="$ros_home"
  roscore -p "$port" >"$run_dir/roscore.log" 2>&1 &
  CORE_PID=$!
  sleep 0.5
  if ! kill -0 "$CORE_PID" 2>/dev/null; then
    echo "failed to start isolated roscore on port $port" >&2
    exit 1
  fi
  for _ in $(seq 1 100); do
    rosparam list >/dev/null 2>&1 && break
    sleep 0.1
  done
  rosparam set /use_sim_time true

  echo "[$label] yaw_rate=$use_yaw_rate ax=$use_acceleration"
  roslaunch bringup localization_gnss_imu_gt_ctra.launch \
    use_sim_time:=true \
    gt_source:=competition \
    use_heading_correction:=true \
    heading_yaw_offset_deg:=-1.347 \
    publish_sensor_tf:=true \
    sensor_config:="$SENSOR_CONFIG" \
    local_ekf_gt_config:="$local_override" \
    global_ekf_gt_config:="$global_override" \
    >"$run_dir/launch.log" 2>&1 &
  LAUNCH_PID=$!
  sleep 2

  rosbag record --lz4 --buffsize=256 -O "$output_bag" \
    /clock /Ego_topic /Competition_topic /sensor/gps /sensor/imu \
    /localization/gnss/odom /localization/odometry/local \
    /localization/odom /localization/kinematic_state /tf /tf_static \
    >"$run_dir/record.log" 2>&1 &
  RECORD_PID=$!
  sleep 1

  rosbag play --clock --rate "$RATE" "$INPUT_BAG" \
    --topics /Ego_topic /Competition_topic /sensor/gps /sensor/imu \
    >"$run_dir/play.log" 2>&1
  sleep 1

  cleanup_run
  python3 "$ANALYZER" "$output_bag" \
    --output "$run_dir/analysis.json" \
    >"$run_dir/analysis.stdout.json"
done

echo "motion input ablation complete: $OUTPUT_ROOT"
