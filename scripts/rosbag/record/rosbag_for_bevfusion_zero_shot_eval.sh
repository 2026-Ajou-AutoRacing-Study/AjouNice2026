#!/usr/bin/env bash
# Record MORAI GT and pre-finetuning BEVFusion outputs into a lightweight evaluation bag.
set -euo pipefail

ROOT="${AJOUNICE_ROOT:-$HOME/AjouNice2026}"
OUT_DIR="${BEVFUSION_EVAL_BAG_DIR:-$ROOT/rosbags/eval/perception}"
RUN_NAME="${1:-$(date '+%m%d_%H%M')}"
BAG="$OUT_DIR/bevfusion_zero_shot_pre_finetune_${RUN_NAME}.bag"
REC_PID=""

TOPICS=(
  /clock
  /Ego_topic
  /Object_topic
  /perception/bevfusion/markers
  /perception/bevfusion/latency_ms
  /perception/bevfusion/status
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
    echo "bag file not found: $BAG"
  fi
  exit "$status"
}

trap finish EXIT
trap 'exit 130' INT TERM

echo "BEVFusion zero-shot evaluation recording"
echo "output: $BAG"
echo "Start raw bag playback only after this recorder says Recording."
printf '  %s\n' "${TOPICS[@]}"

rosbag record --lz4 --buffsize=512 -O "$BAG" "${TOPICS[@]}" &
REC_PID=$!
wait "$REC_PID"
REC_PID=""
