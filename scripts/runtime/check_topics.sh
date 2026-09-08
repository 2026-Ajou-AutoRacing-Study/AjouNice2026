#!/usr/bin/env bash
set -uo pipefail

AJOUNICE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$AJOUNICE_ROOT/scripts/workspace/source_aw149.sh"

check_flow=true
include_outputs=false
wait_seconds="${AW149_TOPIC_WAIT_SECONDS:-5}"

for argument in "$@"; do
  case "$argument" in
    --types-only) check_flow=false ;;
    --full) include_outputs=true ;;
    *)
      echo "usage: $0 [--types-only] [--full]" >&2
      exit 2
      ;;
  esac
done

topics=(
  "/localization/kinematic_state|nav_msgs/Odometry"
  "/localization/acceleration|geometry_msgs/AccelWithCovarianceStamped"
  "/perception/object_recognition/objects|autoware_perception_msgs/PredictedObjects"
  "/perception/traffic_light_recognition/traffic_signals|autoware_perception_msgs/TrafficLightGroupArray"
  "/perception/occupancy_grid_map/map|nav_msgs/OccupancyGrid"
  "/perception/obstacle_segmentation/pointcloud|sensor_msgs/PointCloud2"
  "/map/vector_map|autoware_map_msgs/LaneletMapBin"
  "/map/map_projector_info|autoware_map_msgs/MapProjectorInfo"
  "/system/operation_mode/state|autoware_adapi_v1_msgs/OperationModeState"
  "/api/operation_mode/state|autoware_adapi_v1_msgs/OperationModeState"
)

if $include_outputs; then
  topics+=(
    "/planning/mission_planning/route|autoware_planning_msgs/LaneletRoute"
    "/planning/route_state|autoware_planning_msgs/RouteState"
    "/planning/trajectory|autoware_planning_msgs/Trajectory"
    "/planning/validation_status|autoware_planning_validator/PlanningValidatorStatus"
  )
fi

failures=0
for specification in "${topics[@]}"; do
  topic="${specification%%|*}"
  expected_type="${specification#*|}"
  actual_type="$(rostopic type "$topic" 2>/dev/null || true)"
  topic_info="$(rostopic info "$topic" 2>/dev/null || true)"
  publisher_count="$(printf '%s\n' "$topic_info" | awk '
    /^Publishers:/ { in_publishers=1; next }
    /^Subscribers:/ { in_publishers=0 }
    in_publishers && /^ \* / { count++ }
    END { print count+0 }
  ')"

  if [[ -z "$actual_type" ]]; then
    printf 'MISSING  %-65s expected=%s\n' "$topic" "$expected_type"
    failures=$((failures + 1))
    continue
  fi
  if (( publisher_count == 0 )); then
    printf 'NOPUB    %-65s %s\n' "$topic" "$actual_type"
    failures=$((failures + 1))
    continue
  fi
  if [[ "$actual_type" != "$expected_type" ]]; then
    printf 'TYPEFAIL %-65s expected=%s actual=%s\n' \
      "$topic" "$expected_type" "$actual_type"
    failures=$((failures + 1))
    continue
  fi
  if $check_flow; then
    if timeout "${wait_seconds}s" rostopic echo -n 1 "$topic" >/dev/null 2>&1; then
      printf 'PASS     %-65s %s message=yes\n' "$topic" "$actual_type"
    else
      printf 'NOFLOW   %-65s %s no message in %ss\n' \
        "$topic" "$actual_type" "$wait_seconds"
      failures=$((failures + 1))
    fi
  else
    printf 'PASS     %-65s %s\n' "$topic" "$actual_type"
  fi
done

printf 'AW149 topic preflight: %d failure(s)\n' "$failures"
if (( failures > 0 )); then
  exit 1
fi
