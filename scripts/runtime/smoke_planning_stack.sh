#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export AJOUNICE_ROOT="$REPO_ROOT"

duration_s="${AW149_SMOKE_DURATION_S:-25}"
master_port="${AW149_SMOKE_MASTER_PORT:-11551}"
if (( $# != 0 )); then
  echo "Usage: AW149_SMOKE_DURATION_S=25 AW149_SMOKE_MASTER_PORT=11551 $0" >&2
  exit 2
fi
if ! [[ "$duration_s" =~ ^[1-9][0-9]*$ && "$master_port" =~ ^[1-9][0-9]*$ ]]; then
  echo "ERROR: duration and port must be positive integers." >&2
  exit 2
fi

source "$REPO_ROOT/scripts/workspace/source_aw149.sh"

RUN_DIR="$(mktemp -d /tmp/aw149-planning-smoke.XXXXXX)"
export ROS_HOME="$RUN_DIR/ros_home"
export ROS_MASTER_URI="http://127.0.0.1:$master_port"
export ROS_HOSTNAME=127.0.0.1

cleanup() {
  set +e
  if [[ -n "${launch_pid:-}" ]]; then
    kill -INT "$launch_pid" 2>/dev/null
    wait "$launch_pid" 2>/dev/null
  fi
  if [[ -n "${master_pid:-}" ]]; then
    kill -INT "$master_pid" 2>/dev/null
    wait "$master_pid" 2>/dev/null
  fi
}
trap cleanup EXIT

roscore -p "$master_port" >"$RUN_DIR/roscore.log" 2>&1 &
master_pid=$!
master_ready=false
for _ in $(seq 1 40); do
  if rosparam list >/dev/null 2>&1; then
    master_ready=true
    break
  fi
  sleep 0.25
done
if [[ "$master_ready" != "true" ]]; then
  echo "ERROR: isolated ROS master did not become ready. Logs: $RUN_DIR" >&2
  exit 1
fi

roslaunch ajounice_planning_integration morai_planning_stack.launch \
  start_controller:=false \
  start_requester:=false \
  start_detector:=false \
  >"$RUN_DIR/planning.log" 2>&1 &
launch_pid=$!

sleep "$duration_s"
if ! kill -0 "$launch_pid" 2>/dev/null; then
  echo "ERROR: planning launch exited before ${duration_s}s. Logs: $RUN_DIR" >&2
  tail -100 "$RUN_DIR/planning.log" >&2
  exit 1
fi

expected_nodes=(
  /ego_tf_broadcaster
  /ekf_multi_object_tracking
  /lanelet2_map_loader
  /map_projector_publisher
  /morai_gt_kinematic_state_publisher
  /morai_sensor_static_tf
  /object_prediction
  /operation_mode_state_to_api
  /planning/mission_planning/mission_planner
  /planning/mission_planning/route_selector
  /planning/scenario_planning/external_velocity_limit_selector
  /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner
  /planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner
  /planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner
  /planning/scenario_planning/lane_driving/motion_planning/path_optimizer
  /planning/scenario_planning/lane_driving/motion_planning/path_smoother
  /planning/scenario_planning/scenario_selector
  /planning/scenario_planning/surround_obstacle_checker
  /planning/scenario_planning/velocity_smoother
  /planning_validator
  /tf_map_to_morai_map
  /traffic_light_node
)
mapfile -t actual_nodes < <(rosnode list)
missing_nodes=()
for expected in "${expected_nodes[@]}"; do
  found=false
  for actual in "${actual_nodes[@]}"; do
    if [[ "$actual" == "$expected" ]]; then
      found=true
      break
    fi
  done
  if [[ "$found" != "true" ]]; then
    missing_nodes+=("$expected")
  fi
done

map_type="$(rostopic type /map/vector_map 2>/dev/null || true)"
map_received=false
if timeout 8 rostopic echo -n 1 /map/vector_map --noarr >"$RUN_DIR/vector_map.txt" 2>&1; then
  map_received=true
fi
if command -v rg >/dev/null 2>&1; then
  fatal_lines="$(rg -i -c 'process has died|\[FATAL\]|RLException|Traceback' "$RUN_DIR/planning.log" || true)"
else
  fatal_lines="$(grep -Eic 'process has died|\[FATAL\]|RLException|Traceback' "$RUN_DIR/planning.log" || true)"
fi
fatal_lines="${fatal_lines:-0}"
occupancy_grid_enabled="$(rosparam get /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/use_occupancy_grid 2>/dev/null || true)"
validator_pointcloud_required="$(rosparam get /planning_validator/require_obstacle_pointcloud 2>/dev/null || true)"

printf 'AW149_CLEAN_INSTALL_SMOKE\n'
printf '  duration_s: %s\n' "$duration_s"
printf '  expected_nodes: %d/%d\n' "$(( ${#expected_nodes[@]} - ${#missing_nodes[@]} ))" "${#expected_nodes[@]}"
printf '  map_type: %s\n' "${map_type:-MISSING}"
printf '  map_received: %s\n' "$map_received"
printf '  unexpected_fatal_lines: %s\n' "$fatal_lines"
printf '  occupancy_grid_enabled: %s\n' "${occupancy_grid_enabled:-MISSING}"
printf '  validator_pointcloud_required: %s\n' "${validator_pointcloud_required:-MISSING}"
printf '  logs: %s\n' "$RUN_DIR"

if (( ${#missing_nodes[@]} != 0 )); then
  printf 'ERROR: missing node: %s\n' "${missing_nodes[@]}" >&2
  exit 1
fi
if [[ "$map_type" != "autoware_map_msgs/LaneletMapBin" || "$map_received" != "true" ]]; then
  echo "ERROR: v61 binary map transport was not observed." >&2
  exit 1
fi
if (( fatal_lines != 0 )); then
  echo "ERROR: unexpected fatal signatures in planning.log." >&2
  exit 1
fi
if [[ "$occupancy_grid_enabled" != "false" || "$validator_pointcloud_required" != "false" ]]; then
  echo "ERROR: limited-perception launch policy was not applied." >&2
  exit 1
fi
