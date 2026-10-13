#!/usr/bin/env bash
# Additional lightweight trace; does not launch/restart perception or planning.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
DURATION=60
OUTPUT=""
LIST=false
while (($#)); do
  case "$1" in
    --duration) DURATION="${2:?missing duration}"; shift 2 ;;
    --output) OUTPUT="${2:?missing output}"; shift 2 ;;
    --list) LIST=true; shift ;;
    *) echo "Usage: bash $0 [--duration 60] [--output /path/name.bag] [--list]" >&2; exit 2 ;;
  esac
done
if [[ ! "$DURATION" =~ ^[1-9][0-9]*$ ]] || ((DURATION>600)); then
  echo "duration must be 1..600 seconds" >&2; exit 2
fi
TOPICS=(
  '^/perception/bevfusion/(status|latency_ms|frame_timing)$'
  '^/perception/base5_trial/(status|latency_ms|frame_timing)$'
  '^/perception/object_recognition/(detection/objects|tracking/measurement_objects|tracking/objects|objects)$'
  '^/map_based_prediction/diagnostics/processing_time_ms$'
  '^/tracked_object_motion_policy/diagnostics/processing_time_ms$'
  '^/planning/trajectory$'
  '^/planning/validation_status$'
  '^/planning/.*/debug/(processing_time.*|calculation_time)$'
  '^/planning/.*/(processing_time_ms|debug/processing_time_ms_diag)$'
  '^/localization/kinematic_state$'
  '^/diagnostics$'
  '^/rosout$'
  '^/tf$'
  '^/tf_static$'
  '^/clock$'
)
if $LIST; then
  printf '%s\n' "${TOPICS[@]}"
  exit 0
fi
OUTPUT="${OUTPUT:-$ROOT/rosbags/eval/perception/node_latency/latency_$(date +%Y%m%d_%H%M%S).bag}"
if [[ -e "$OUTPUT" || -e "$OUTPUT.active" ]]; then
  echo "Refusing to overwrite $OUTPUT" >&2; exit 2
fi
mkdir -p "$(dirname "$OUTPUT")"
echo "추가 지연 진단 녹화: $OUTPUT ($DURATION 초). 원본 센서는 녹화하지 않습니다."
echo "토픽이 발행되지 않거나 디버그가 꺼져 있으면 저장되지 않습니다. 기존 노드는 변경하지 않습니다."
exec rosbag record --lz4 --buffsize=256 --duration="$DURATION" --regex -O "$OUTPUT" "${TOPICS[@]}"
