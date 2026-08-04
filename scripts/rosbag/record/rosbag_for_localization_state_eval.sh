#!/usr/bin/env bash
set -euo pipefail

# Record a self-contained localization pose/twist evaluation bag while
# replaying a raw MORAI bag. Start the localization stack and this recorder
# first, then unpause rosbag play in another terminal.

ROOT="${AJOUNICE_ROOT:-$HOME/AjouNice2026}"
OUT_DIR="${ROSBAG_EVAL_DIR:-$ROOT/rosbags/eval}"
STAMP="$(date '+%m%d_%H%M')"
BAG="${LOCALIZATION_STATE_EVAL_BAG:-$OUT_DIR/localization_state_full_$STAMP.bag}"
REC_PID=""

TOPICS=(
  /clock
  /Ego_topic
  /sensor/gps
  /sensor/imu
  /sensor/lidar1
  /localization/odom
  /localization/kinematic_state
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

echo "Localization state evaluation recording"
echo "output: $BAG"
echo "topics:"
printf '  %s\n' "${TOPICS[@]}"
echo "Start or unpause the raw bag after 'Recording to' appears."
echo "Press Ctrl-C after rosbag play finishes."

rosbag record --lz4 --buffsize=4096 -O "$BAG" "${TOPICS[@]}" &
REC_PID=$!
wait "$REC_PID"
REC_PID=""
