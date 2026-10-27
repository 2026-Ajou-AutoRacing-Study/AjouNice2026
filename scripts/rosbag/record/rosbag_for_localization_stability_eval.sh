#!/usr/bin/env bash
set -euo pipefail

# Record one complete MR drive for localization accuracy and control-chattering
# analysis. /Ego_topic is recorded only as an offline oracle; the production
# localization stack must still run with gt_source:=competition.

ROOT="${AJOUNICE_ROOT:-$HOME/AjouNice2026}"
OUT_DIR="${LOCALIZATION_STABILITY_BAG_DIR:-$ROOT/rosbags/eval/localization/local_mr_stability}"
STAMP="$(date '+%y%m%d_%H%M%S')"
RUN_TAG="${1:-full_route}"
BAG="${LOCALIZATION_STABILITY_BAG:-$OUT_DIR/${STAMP}_${RUN_TAG}.bag}"
REC_PID=""

TOPICS=(
  # Time and MORAI references.
  /clock
  /Ego_topic
  /Competition_topic
  /competition/raw_twist
  /sensor/gps
  /sensor/imu

  # Localization inputs after adaptation, intermediate filters, and outputs.
  /localization/imu
  /localization/gt/orientation
  /localization/gt/twist
  /localization/gnss/odom
  /localization/odometry/local
  /localization/odom
  /localization/kinematic_state
  /localization/control_kinematic_state
  /localization/acceleration

  # Object and traffic-light context that can change the planned trajectory.
  /Object_topic
  /IntscnTL_topic
  /GetTrafficLightStatus
  /perception/object_recognition/detection/objects
  /perception/object_recognition/tracking/measurement_objects
  /perception/object_recognition/tracking/objects
  /perception/object_recognition/objects
  /sensor/camera/front/compressed
  /perception/traffic_light_recognition/route_group_id
  /perception/traffic_light_recognition/map_rois
  /perception/traffic_light_recognition/traffic_signals
  /traffic_light_node/debug_selection
  /traffic_light_node/debug_classifier
  /traffic_light_node/debug_runtime
  /traffic_light_node/debug_policy_reason
  /traffic_light_node/debug_observed_label
  /traffic_light_node/debug_raw_label

  # Final and nearby planning outputs needed to align controller behavior.
  /planning/mission_planning/route
  /planning/route_state
  /planning/scenario_planning/lane_driving/behavior_planning/path
  /planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id
  /planning/scenario_planning/lane_driving/behavior_planning/static_obstacle_scene
  /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/avoidance_debug_message_array
  /planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory
  /planning/trajectory
  /planning/validation_status

  # Controller outputs and diagnostics used by the chattering benchmark.
  /control/autoware/controller_valid
  /control/autoware/failure_reason
  /control/autoware/target_point
  /control/command/control_cmd
  /control/command/steering
  /purepursuit_autoware/debug/controller_state
  /purepursuit_autoware/debug/lateral_error
  /purepursuit_autoware/debug/lookahead_distance
  /purepursuit_autoware/debug/target_curvature
  /purepursuit_autoware/debug/limited_steering_angle
  /ctrl_cmd

  # Frame graph, simulator state, and runtime diagnostics.
  /CollisionData
  /VehicleCollisionData
  /sim/process/state/msg/MoraiSimProcStatus
  /tf
  /tf_static
  /diagnostics
  /rosout
  /rosout_agg
)

mkdir -p "$(dirname "$BAG")"

METADATA="${BAG%.bag}.metadata.txt"
{
  echo "recorded_at: $(date --iso-8601=seconds)"
  echo "git_commit: $(git -C "$ROOT" rev-parse HEAD 2>/dev/null || echo unknown)"
  echo "git_branch: $(git -C "$ROOT" branch --show-current 2>/dev/null || echo unknown)"
  echo "use_sim_time: $(rosparam get /use_sim_time 2>/dev/null || echo unavailable)"
  echo "controller_kinematic_state_topic: $(rosparam get /purepursuit_autoware/topics/kinematic_state 2>/dev/null || echo unavailable)"
  echo "localization_provider_params:"
  rosparam get /localization_manager 2>/dev/null || echo "  unavailable"
  echo "smooth_control_state_params:"
  rosparam get /smooth_control_state_publisher 2>/dev/null || echo "  disabled_or_unavailable"
  echo "active_nodes:"
  rosnode list 2>/dev/null || echo "  unavailable"
} >"$METADATA"

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

echo "MR localization stability evaluation recording"
echo "output: $BAG"
echo "metadata: $METADATA"
echo "run tag: $RUN_TAG"
echo "topics:"
printf '  %s\n' "${TOPICS[@]}"
echo
echo "Start this recorder after UDP, localization, and the planning/controller stack are healthy."
echo "Start the route only after 'Recording to' appears."
echo "Stop safely with Ctrl-C after the vehicle has completed the route."

rosbag record --lz4 --buffsize=4096 -O "$BAG" "${TOPICS[@]}" &
REC_PID=$!
wait "$REC_PID"
REC_PID=""
