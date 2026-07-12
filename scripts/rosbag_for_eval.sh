#!/usr/bin/env bash
set -euo pipefail

ROOT="${AJOUNICE_ROOT:-$HOME/AjouNice2026}"
OUT_DIR="${ROSBAG_EVAL_DIR:-$ROOT/rosbags/eval}"
STAMP="$(date '+%m%d_%H%M')"
BAG="$OUT_DIR/fast_lio_full_$STAMP.bag"
REC_PID=""

TOPICS=(
  /clock
  /Ego_topic
  /sensor/imu
  /sensor/lidar1
  /localization/odom
  /path
  /cloud_registered
  /tf
)

mkdir -p "$OUT_DIR"

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
    echo "bag file not found yet: $BAG"
  fi

  exit "$status"
}

trap finish EXIT
trap 'exit 130' INT TERM

echo "record start: $(date '+%Y-%m-%d %H:%M:%S')"
echo "output: $BAG"
echo "topics:"
printf '  %s\n' "${TOPICS[@]}"
echo "press Ctrl-C to stop recording"

rosbag record --lz4 --buffsize=4096 -O "$BAG" "${TOPICS[@]}" &
REC_PID=$!
wait "$REC_PID"
REC_PID=""