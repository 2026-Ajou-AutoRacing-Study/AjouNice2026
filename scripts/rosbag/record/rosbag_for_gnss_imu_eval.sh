#!/usr/bin/env bash
set -euo pipefail

# Record GNSS+IMU localization inputs, intermediate measurements, outputs, and TF.

ROOT="${AJOUNICE_ROOT:-$HOME/AjouNice2026}"
OUT_DIR="${ROSBAG_EVAL_DIR:-$ROOT/rosbags/eval/localization}"
STAMP="$(date '+%m%d_%H%M')"
BAG="${GNSS_IMU_EVAL_BAG:-$OUT_DIR/gnss_imu_full_$STAMP.bag}"
REC_PID=""

TOPICS=(
  /clock
  /Ego_topic
  /Competition_topic
  /competition/raw_twist
  /sensor/gps
  /sensor/imu
  /localization/imu
  /localization/gt/orientation
  /localization/gt/twist
  /localization/gnss/odom
  /localization/odometry/local
  /localization/odom
  /localization/kinematic_state
  /localization/acceleration
  /tf
  /tf_static
)

mkdir -p "$(dirname "$BAG")"

finish() {
  local status=$?
  if [[ -n "$REC_PID" ]] && kill -0 "$REC_PID" 2>/dev/null; then
    kill -INT "$REC_PID" 2>/dev/null || true
    wait "$REC_PID" 2>/dev/null || true
  fi

  echo
  echo "record done: $(date '+%Y-%m-%d %H:%M:%S')"
  if [[ -f "$BAG" ]]; then
    rosbag info "$BAG"
  else
    echo "bag file not found: $BAG"
  fi
  exit "$status"
}

trap finish EXIT
trap 'exit 130' INT TERM

echo "GNSS+IMU robot_localization evaluation recording"
echo "output: $BAG"
echo "topics:"
printf '  %s\n' "${TOPICS[@]}"
echo "Start or unpause MORAI/raw bag after 'Recording to' appears."
echo "Press Ctrl-C after the run finishes."

rosbag record --lz4 --buffsize=1024 -O "$BAG" "${TOPICS[@]}" &
REC_PID=$!
wait "$REC_PID"
REC_PID=""
