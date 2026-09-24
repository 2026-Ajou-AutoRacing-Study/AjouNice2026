#!/usr/bin/env bash
set -euo pipefail

# Lightweight closed-loop evidence for BEVFusion -> tracking -> planning
# static-obstacle failures. Raw camera/LiDAR payloads are intentionally not
# recorded; their live UDP topics remain enabled for BEVFusion inference.

ROOT="${AJOUNICE_ROOT:-$HOME/AjouNice2026}"
OUT_DIR="${STATIC_OBSTACLE_TRACKING_BAG_DIR:-$ROOT/rosbags/eval/perception/static_obstacle_tracking}"
STAMP="$(date '+%y%m%d_%H%M%S')"
RUN_TAG="${1:-bevfusion_static_obstacle}"
BAG="${STATIC_OBSTACLE_TRACKING_BAG:-$OUT_DIR/${STAMP}_${RUN_TAG}.bag}"
PREFIX="${BAG%.bag}"
REC_PID=""
RECORD_RAW_SENSORS="${STATIC_OBSTACLE_RECORD_RAW_SENSORS:-false}"

# Preserve every structured perception/planning/control topic so changes in
# ported Autoware topic names do not silently remove evidence. Sensor payloads
# live under /sensor and therefore do not match this expression.
INCLUDE_REGEX='^/(clock|Ego_topic|Competition_topic|Object_topic|CollisionData|VehicleCollisionData|tf|tf_static|diagnostics|rosout|rosout_agg|ctrl_cmd)$|^/(localization|perception|planning|control|competition_route_requester|api|autoware)/.*'

# Full-mode adds exactly the live BEVFusion sensor inputs plus GPS/IMU.  Keep
# the lightweight structured-topic contract unchanged so the two recordings
# can be compared without changing the downstream evidence.
RAW_SENSOR_REGEX='^/sensor/(lidar1|gps|imu)$|^/sensor/camera/(front|front_left|front_right|back)/compressed$'
if [[ "$RECORD_RAW_SENSORS" == "true" ]]; then
  INCLUDE_REGEX="${INCLUDE_REGEX}|${RAW_SENSOR_REGEX}"
  RECORDING_MODE="full_with_camera_lidar"
  BUFFSIZE_MB=4096
else
  RECORDING_MODE="lightweight_no_camera_lidar"
  BUFFSIZE_MB=1024
fi

# RViz/debug streams duplicate structured messages and can dominate a bag.
# More importantly, merely subscribing to some lazy debug publishers changes
# node execution: motion_velocity_planner's processed-pointcloud publisher is
# one such path.  Do not rely on rosbag's runtime --exclude filtering here;
# resolve the safe topic snapshot before starting the recorder and pass only
# those exact names to rosbag.
EXCLUDE_REGEX='(^/perception/object_recognition/visualization(/|$))|(^/perception/bevfusion/projection(/|$))|(^/perception/obstacle_segmentation/pointcloud$)|(^/planning/debug(/|$))|(^/planning/.*/debug(/|$))|(^/planning/.*/(marker|markers)(/|$))'

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

{
  echo "started_at=$(date --iso-8601=seconds)"
  echo "run_tag=$RUN_TAG"
  echo "hostname=$(hostname)"
  echo "repo_root=$ROOT"
  echo "git_branch=$(git -C "$ROOT" branch --show-current 2>/dev/null || true)"
  echo "git_sha=$(git -C "$ROOT" rev-parse HEAD 2>/dev/null || true)"
  echo "recording_mode=$RECORDING_MODE"
  echo "include_regex=$INCLUDE_REGEX"
  echo "exclude_regex=$EXCLUDE_REGEX"
} >"${PREFIX}_manifest.txt"

rostopic list >"${PREFIX}_topics_at_start.txt" 2>&1 || true
rosnode list >"${PREFIX}_nodes_at_start.txt" 2>&1 || true
rosparam dump "${PREFIX}_rosparams.yaml" >/dev/null 2>&1 || true

mapfile -t RECORD_TOPICS < <(
  grep -E "$INCLUDE_REGEX" "${PREFIX}_topics_at_start.txt" \
    | grep -Ev "$EXCLUDE_REGEX" \
    | sort -u
)
if (( ${#RECORD_TOPICS[@]} == 0 )); then
  echo "No matching ROS topics are currently advertised." >&2
  exit 2
fi
printf '%s\n' "${RECORD_TOPICS[@]}" >"${PREFIX}_recorded_topics.txt"

# Guard against reintroducing the subscriber-triggered planner failure.
if printf '%s\n' "${RECORD_TOPICS[@]}" \
    | grep -Eq '/motion_velocity_planner/debug/processed_pointcloud$'; then
  echo "Refusing to subscribe to motion_velocity_planner debug pointcloud." >&2
  exit 2
fi

echo "Static-obstacle tracking/planning evaluation recording"
echo "output: $BAG"
echo "mode: $RECORDING_MODE"
echo "selected topics: ${#RECORD_TOPICS[@]}"
if [[ "$RECORD_RAW_SENSORS" == "true" ]]; then
  echo "Camera, LiDAR, GPS and IMU topics ARE recorded."
else
  echo "Raw camera and LiDAR topics are NOT recorded."
fi
echo "Start driving only after 'Recording to' appears."
echo "Stop with Ctrl-C after the failure or successful avoidance is complete."

rosbag record \
  --lz4 \
  --buffsize="$BUFFSIZE_MB" \
  -O "$BAG" \
  "${RECORD_TOPICS[@]}" &
REC_PID=$!
wait "$REC_PID"
REC_PID=""
