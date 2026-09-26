#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 INPUT_BAG" >&2
  exit 2
fi

ROOT="${AJOUNICE_ROOT:-$HOME/AjouNice2026}"
INPUT_BAG="$(readlink -f "$1")"
RATE="${LOCALIZATION_GNSS_INPUT_AB_RATE:-1.0}"
PORT_BASE="${LOCALIZATION_GNSS_INPUT_AB_PORT_BASE:-$((30000 + ($$ % 5000)))}"
STAMP="$(date '+%y%m%d_%H%M%S')"
OUTPUT_ROOT="${LOCALIZATION_GNSS_INPUT_AB_OUTPUT_DIR:-$HOME/localization_experiments/gnss_input_ab_$STAMP}"
SENSOR_CONFIG="$ROOT/configs/sensors/Sensor_competition_local.json"
ANALYZER="$ROOT/scripts/evaluation/localization/analyze_localization_stability_bag.py"

if [[ ! -f "$INPUT_BAG" ]]; then
  echo "input bag not found: $INPUT_BAG" >&2
  exit 2
fi

mkdir -p "$OUTPUT_ROOT"
source /opt/ros/noetic/setup.bash
source "$ROOT/scripts/workspace/source_aw149.sh"

CORE_PID=""; LAUNCH_PID=""; RECORD_PID=""
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
  stop_process "$RECORD_PID"; stop_process "$LAUNCH_PID"; stop_process "$CORE_PID"
  RECORD_PID=""; LAUNCH_PID=""; CORE_PID=""
}
trap cleanup_run EXIT INT TERM

if [[ -n "${LOCALIZATION_GNSS_INPUT_AB_PROFILES:-}" ]]; then
  read -r -a profiles <<<"$LOCALIZATION_GNSS_INPUT_AB_PROFILES"
else
  profiles=(off cell_transition)
fi

for index in "${!profiles[@]}"; do
  mode="${profiles[$index]}"
  if [[ "$mode" != "off" && "$mode" != "cell_transition" ]]; then
    echo "invalid GNSS input mode: $mode" >&2
    exit 2
  fi
  run_dir="$OUTPUT_ROOT/$mode"; mkdir -p "$run_dir"
  output_bag="$run_dir/localization_${mode}.bag"
  port=$((PORT_BASE + index))
  export ROS_MASTER_URI="http://127.0.0.1:$port"
  export ROS_HOME="/tmp/ajou_gnss_input_${STAMP}_${mode}"
  mkdir -p "$ROS_HOME"

  printf '%s\n' \
    "input_bag=$INPUT_BAG" \
    "source_commit=$(git -C "$ROOT" rev-parse HEAD)" \
    "worktree_diff=$(git -C "$ROOT" status --short | tr '\n' ';')" \
    "mode=$mode" "replay_rate=$RATE" \
    "gt_source=competition" "estimator_heading_offset_deg=-1.347" \
    "evaluator_heading_offset_deg=-1.347" "time_policy=recorded_header" \
    >"$run_dir/metadata.txt"

  roscore -p "$port" >"$run_dir/roscore.log" 2>&1 & CORE_PID=$!
  for _ in $(seq 1 100); do rosparam list >/dev/null 2>&1 && break; sleep 0.1; done
  rosparam set /use_sim_time true

  roslaunch bringup localization_gnss_imu_gt_ctra.launch \
    use_sim_time:=true gt_source:=competition \
    use_heading_correction:=true heading_yaw_offset_deg:=-1.347 \
    publish_sensor_tf:=true sensor_config:="$SENSOR_CONFIG" \
    publish_control_state:=true \
    gnss_quantization_input_mode:="$mode" \
    >"$run_dir/launch.log" 2>&1 & LAUNCH_PID=$!
  sleep 2
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "localization launch failed; see $run_dir/launch.log" >&2
    exit 1
  fi

  rosbag record --lz4 --buffsize=256 -O "$output_bag" \
    /clock /Ego_topic /Competition_topic /sensor/gps /sensor/imu \
    /localization/gnss/odom /localization/odometry/local \
    /localization/odom /localization/kinematic_state \
    /localization/control_kinematic_state /tf /tf_static \
    >"$run_dir/record.log" 2>&1 & RECORD_PID=$!
  sleep 1

  echo "[$mode] replaying $INPUT_BAG"
  rosbag play --clock --rate "$RATE" "$INPUT_BAG" \
    --topics /Ego_topic /Competition_topic /sensor/gps /sensor/imu /tf_static \
    >"$run_dir/play.log" 2>&1
  sleep 1
  cleanup_run

  python3 "$ANALYZER" "$output_bag" --output "$run_dir/analysis.json" \
    >"$run_dir/analysis.stdout.json"
done

echo "GNSS input A/B complete: $OUTPUT_ROOT"
