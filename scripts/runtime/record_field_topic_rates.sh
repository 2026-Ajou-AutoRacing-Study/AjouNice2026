#!/usr/bin/env bash
# Record a simultaneous field snapshot of the streaming ROS topic rates.

set -uo pipefail

AJOUNICE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DURATION_SEC="${1:-20}"
OUTPUT_ROOT="${2:-$AJOUNICE_ROOT/artifacts/field_checks/topic_hz}"

if ! [[ "$DURATION_SEC" =~ ^[1-9][0-9]*$ ]]; then
  echo "usage: $0 [duration_sec] [output_root]" >&2
  exit 2
fi

source "$AJOUNICE_ROOT/scripts/workspace/source_aw149.sh"

if ! rosparam get /rosversion >/dev/null 2>&1; then
  echo "ERROR: cannot reach ROS master at ${ROS_MASTER_URI:-unset}" >&2
  exit 1
fi

RUN_ID="$(date +%y%m%d_%H%M%S)"
OUTPUT_DIR="$OUTPUT_ROOT/$RUN_ID"
RATE_DIR="$OUTPUT_DIR/rates"
INFO_DIR="$OUTPUT_DIR/info"
mkdir -p "$RATE_DIR" "$INFO_DIR"

topics=(
  /clock
  /sensor/lidar1/packets
  /sensor/lidar1
  /sensor/camera/front/compressed
  /sensor/camera/front_left/compressed
  /sensor/camera/front_right/compressed
  /sensor/camera/back/compressed
  /sensor/gps
  /sensor/imu
  /Ego_topic
  /Competition_topic
  /Object_topic
  /localization/kinematic_state
  /localization/acceleration
  /perception/object_recognition/detection/objects
  /perception/object_recognition/tracking/measurement_objects
  /perception/object_recognition/tracking/objects
  /perception/object_recognition/objects
  /perception/traffic_light_recognition/route_group_id
  /perception/traffic_light_recognition/traffic_signals
  /planning/scenario_planning/lane_driving/behavior_planning/path
  /planning/trajectory
  /planning/validation_status
  /control/command/steering
  /ctrl_cmd
  /tf
)

sanitize_topic() {
  local topic="$1"
  topic="${topic#/}"
  printf '%s' "${topic//\//__}"
}

{
  echo "run_id=$RUN_ID"
  echo "started_at=$(date --iso-8601=seconds)"
  echo "hostname=$(hostname)"
  echo "duration_sec=$DURATION_SEC"
  echo "ros_master_uri=${ROS_MASTER_URI:-unset}"
  echo "git_branch=$(git -C "$AJOUNICE_ROOT" branch --show-current 2>/dev/null || true)"
  echo "git_sha=$(git -C "$AJOUNICE_ROOT" rev-parse HEAD 2>/dev/null || true)"
  echo "output_dir=$OUTPUT_DIR"
  echo
  ip -br addr 2>&1 || true
} >"$OUTPUT_DIR/metadata.txt"

rosnode list 2>&1 | sort >"$OUTPUT_DIR/rosnodes.txt" || true
rostopic list 2>&1 | sort >"$OUTPUT_DIR/rostopics.txt" || true

declare -a pids=()
for topic in "${topics[@]}"; do
  name="$(sanitize_topic "$topic")"
  rostopic info "$topic" >"$INFO_DIR/$name.txt" 2>&1 || true
  (
    # SIGINT makes rostopic hz print its final statistics before exiting.
    timeout --signal=INT --kill-after=3s "${DURATION_SEC}s" \
      rostopic hz --wall-time "$topic" >"$RATE_DIR/$name.txt" 2>&1 || true
  ) &
  pids+=("$!")
done

echo "Measuring ${#topics[@]} topics simultaneously for ${DURATION_SEC}s ..."
for pid in "${pids[@]}"; do
  wait "$pid" || true
done

summary="$OUTPUT_DIR/summary.tsv"
printf 'topic\tstatus\tlast_average_hz\tpublisher_count\n' >"$summary"
for topic in "${topics[@]}"; do
  name="$(sanitize_topic "$topic")"
  rate_file="$RATE_DIR/$name.txt"
  info_file="$INFO_DIR/$name.txt"
  last_rate="$(awk '/average rate:/ { value=$3 } END { print value }' "$rate_file")"
  publisher_count="$(awk '
    /^Publishers:/ { active=1; next }
    /^Subscribers:/ { active=0 }
    active && /^ \* / { count++ }
    END { print count+0 }
  ' "$info_file")"

  if [[ -n "$last_rate" ]]; then
    status="OK"
  elif (( publisher_count == 0 )); then
    status="NO_PUBLISHER"
  else
    status="NO_MESSAGES"
  fi
  printf '%s\t%s\t%s\t%s\n' \
    "$topic" "$status" "${last_rate:--}" "$publisher_count" >>"$summary"
done

{
  echo "finished_at=$(date --iso-8601=seconds)"
  df -h "$OUTPUT_DIR" 2>&1 || true
} >>"$OUTPUT_DIR/metadata.txt"

echo
column -t -s $'\t' "$summary" 2>/dev/null || cat "$summary"
echo
echo "Saved field topic-rate snapshot: $OUTPUT_DIR"
echo "$OUTPUT_DIR" > /tmp/ajounice_latest_topic_hz_dir
