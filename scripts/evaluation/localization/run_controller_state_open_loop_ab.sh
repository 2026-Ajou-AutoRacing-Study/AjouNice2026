#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 INPUT_BAG" >&2
  exit 2
fi

ROOT="${AJOUNICE_ROOT:-$HOME/AjouNice2026}"
INPUT_BAG="$(readlink -f "$1")"
RATE="${LOCALIZATION_CONTROL_AB_RATE:-1.0}"
PORT_BASE="${LOCALIZATION_CONTROL_AB_PORT_BASE:-$((28000 + ($$ % 8000)))}"
STAMP="$(date '+%y%m%d_%H%M%S')"
OUTPUT_ROOT="${LOCALIZATION_CONTROL_AB_OUTPUT_DIR:-$ROOT/rosbags/eval/localization/controller_state_open_loop_$STAMP}"
ANALYZER="$ROOT/scripts/evaluation/localization/analyze_localization_stability_bag.py"

if [[ ! -f "$INPUT_BAG" ]]; then
  echo "input bag not found: $INPUT_BAG" >&2
  exit 2
fi

mkdir -p "$OUTPUT_ROOT"
source /opt/ros/noetic/setup.bash
source "$ROOT/scripts/workspace/source_aw149.sh"

CORE_PID=""
SMOOTH_PID=""
CONTROL_PID=""
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
  stop_process "$CONTROL_PID"
  stop_process "$SMOOTH_PID"
  stop_process "$CORE_PID"
  RECORD_PID=""
  CONTROL_PID=""
  SMOOTH_PID=""
  CORE_PID=""
}
trap cleanup_run EXIT INT TERM

read -r -a smooth_taus <<<"${LOCALIZATION_CONTROL_AB_TAUS:-0.5}"
profiles=()
if [[ "${LOCALIZATION_CONTROL_AB_INCLUDE_GLOBAL:-true}" == "true" ]]; then
  profiles+=("global:0")
fi
for tau in "${smooth_taus[@]}"; do
  profiles+=("smooth_tau${tau/./p}:$tau")
done

for index in "${!profiles[@]}"; do
  IFS=: read -r label tau <<<"${profiles[$index]}"
  if [[ "$label" == "global" ]]; then
    label="global"
    state_topic="/localization/kinematic_state"
  else
    state_topic="/localization/control_kinematic_state"
  fi
  run_dir="$OUTPUT_ROOT/$label"
  mkdir -p "$run_dir"
  output_bag="$run_dir/controller_${label}.bag"
  port=$((PORT_BASE + index))
  ros_home="/tmp/ajou_control_state_ab_${STAMP}_${label}"
  rm -rf "$ros_home"
  mkdir -p "$ros_home"
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

  if [[ "$label" != "global" ]]; then
    rosrun localization_manager smooth_control_state_publisher.py \
      _translation_tau_sec:="$tau" \
      _yaw_tau_sec:="$tau" \
      _reset_translation_error_m:=5.0 \
      _reset_yaw_error_deg:=20.0 \
      >"$run_dir/smooth.log" 2>&1 &
    SMOOTH_PID=$!
  fi

  roslaunch trajectory_follower pp_pi_autoware.launch \
    kinematic_state_topic:="$state_topic" \
    >"$run_dir/controller.log" 2>&1 &
  CONTROL_PID=$!
  sleep 2

  rosbag record --lz4 --buffsize=256 -O "$output_bag" \
    /clock /Ego_topic /Competition_topic \
    /localization/gnss/odom /localization/odometry/local \
    /localization/kinematic_state /localization/control_kinematic_state \
    /planning/trajectory \
    /control/autoware/controller_valid /control/autoware/failure_reason \
    /control/autoware/target_point /control/command/steering \
    /purepursuit_autoware/debug/controller_state \
    /purepursuit_autoware/debug/lateral_error \
    /purepursuit_autoware/debug/lookahead_distance \
    /purepursuit_autoware/debug/target_curvature \
    /purepursuit_autoware/debug/limited_steering_angle \
    /ctrl_cmd /tf_static \
    >"$run_dir/record.log" 2>&1 &
  RECORD_PID=$!
  sleep 1

  echo "[$label] controller input=$state_topic"
  rosbag play --clock --rate "$RATE" "$INPUT_BAG" \
    --topics /Ego_topic /Competition_topic /localization/gnss/odom \
    /localization/odometry/local /localization/kinematic_state \
    /planning/trajectory /tf_static \
    >"$run_dir/play.log" 2>&1
  sleep 1

  cleanup_run
  python3 "$ANALYZER" "$output_bag" \
    --output "$run_dir/analysis.json" \
    >"$run_dir/analysis.stdout.json"
done

echo "controller-state open-loop A/B complete: $OUTPUT_ROOT"
