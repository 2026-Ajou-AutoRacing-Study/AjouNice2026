#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "usage: $0 INPUT_BAG [SIGMA_M ...]" >&2
  exit 2
fi

ROOT="${AJOUNICE_ROOT:-$HOME/AjouNice2026}"
INPUT_BAG="$(readlink -f "$1")"
shift
SIGMAS=("${@:-0.05 0.10 0.20 0.30 0.50}")
RATE="${LOCALIZATION_SWEEP_RATE:-2.0}"
# Use a per-process port by default.  Reusing a fixed port can silently attach a
# new sweep to an orphaned roscore and mix two /clock publishers, corrupting the
# EKF replay without making rosbag play fail.
PORT="${LOCALIZATION_SWEEP_PORT:-$((20000 + ($$ % 20000)))}"
STAMP="$(date '+%y%m%d_%H%M%S')"
OUTPUT_ROOT="${LOCALIZATION_SWEEP_OUTPUT_DIR:-$ROOT/rosbags/eval/localization/gnss_covariance_sweep_$STAMP}"
BASE_NOISE="$ROOT/configs/localization/gnss_imu_gt_noise_sensor6_ideal.yaml"
SENSOR_CONFIG="$ROOT/configs/sensors/Sensor_competition_local.json"
ANALYZER="$ROOT/scripts/evaluation/localization/analyze_localization_stability_bag.py"

if [[ ! -f "$INPUT_BAG" ]]; then
  echo "input bag not found: $INPUT_BAG" >&2
  exit 2
fi

mkdir -p "$OUTPUT_ROOT"

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

source /opt/ros/noetic/setup.bash
source "$ROOT/scripts/workspace/source_aw149.sh"

for sigma in ${SIGMAS[*]}; do
  label="sigma_${sigma/./p}"
  run_dir="$OUTPUT_ROOT/$label"
  mkdir -p "$run_dir"
  noise_config="$run_dir/noise.yaml"
  output_bag="$run_dir/localization_${label}.bag"
  ros_home="/tmp/ajou_gnss_covariance_${STAMP}_${label}"
  rm -rf "$ros_home"
  mkdir -p "$ros_home"

  sed -E \
    "s/^(  horizontal_stddev_m:).*/\\1 $sigma/" \
    "$BASE_NOISE" > "$noise_config"

  export ROS_MASTER_URI="http://127.0.0.1:$PORT"
  export ROS_HOME="$ros_home"

  echo
  echo "[$label] starting isolated ROS master on port $PORT"
  roscore -p "$PORT" >"$run_dir/roscore.log" 2>&1 &
  CORE_PID=$!
  sleep 0.5
  if ! kill -0 "$CORE_PID" 2>/dev/null; then
    echo "failed to start isolated roscore on port $PORT; refusing to reuse an existing master" >&2
    cat "$run_dir/roscore.log" >&2 || true
    exit 1
  fi
  for _ in $(seq 1 100); do
    rosparam list >/dev/null 2>&1 && break
    sleep 0.1
  done
  if ! kill -0 "$CORE_PID" 2>/dev/null; then
    echo "isolated roscore exited during startup" >&2
    exit 1
  fi
  rosparam set /use_sim_time true

  echo "[$label] starting localization, GNSS horizontal sigma=$sigma m"
  roslaunch bringup localization_gnss_imu_gt_ctra.launch \
    use_sim_time:=true \
    gt_source:=competition \
    use_heading_correction:=true \
    heading_yaw_offset_deg:=-1.347 \
    publish_sensor_tf:=true \
    sensor_config:="$SENSOR_CONFIG" \
    measurement_noise_config:="$noise_config" \
    >"$run_dir/launch.log" 2>&1 &
  LAUNCH_PID=$!
  sleep 2

  echo "[$label] recording localization output"
  rosbag record --lz4 --buffsize=1024 -O "$output_bag" \
    /clock \
    /Ego_topic \
    /Competition_topic \
    /sensor/gps \
    /sensor/imu \
    /localization/imu \
    /localization/gt/orientation \
    /localization/gt/twist \
    /localization/gnss/odom \
    /localization/odometry/local \
    /localization/odom \
    /localization/kinematic_state \
    /localization/acceleration \
    /tf \
    /tf_static \
    >"$run_dir/record.log" 2>&1 &
  RECORD_PID=$!
  sleep 2

  echo "[$label] replaying 522 s bag at ${RATE}x"
  rosbag play --clock --rate "$RATE" "$INPUT_BAG" \
    --topics /Ego_topic /Competition_topic /sensor/gps /sensor/imu \
    >"$run_dir/play.log" 2>&1
  sleep 2

  stop_process "$RECORD_PID"
  RECORD_PID=""
  stop_process "$LAUNCH_PID"
  LAUNCH_PID=""
  stop_process "$CORE_PID"
  CORE_PID=""

  echo "[$label] analyzing output"
  python3 "$ANALYZER" "$output_bag" \
    --output "$run_dir/analysis.json" \
    >"$run_dir/analysis.stdout.json"
  rosbag info "$output_bag" >"$run_dir/rosbag_info.txt"
  echo "[$label] complete"
done

echo
echo "GNSS covariance sweep complete: $OUTPUT_ROOT"
