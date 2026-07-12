# Behavior Path Planner ROS1 Port

This package now provides two BPP execution styles:

| Launch file | Shape | Use case |
| --- | --- | --- |
| `behavior_path_planner.launch` | One integrated BPP node | Autoware-style runtime path: lane following, lane change, obstacle avoidance, start/goal, and side shift are applied inside one planning cycle. |
| `validation/launch/behavior_path_planner_mvp.launch` | Multiple simple nodes connected by topics | Validation-only module inspection; excluded from production installs. |

The competition launch disables Goal Planner and Side Shift by default because
there is no parking mission or external lateral-offset input. Their source and
tests remain available for future use. They can be re-enabled explicitly with
`enable_goal_planner:=true` or `enable_side_shift:=true`.

## Integrated Node

`scripts/behavior_path_planner.py` is the main ROS1 port target for Behavior Path Planner.

It subscribes to:

| ROS1 topic | Message | Purpose |
| --- | --- | --- |
| `/planning/mission_planning/route` | `autoware_planning_msgs/LaneletRoute` | Route and lane IDs. |
| `/map/vector_map` | `autoware_map_msgs/LaneletMapBin` | Optional raw OSM map payload. Binary Lanelet2 payload parsing is not available in this Python port. |
| `/localization/kinematic_state` | `nav_msgs/Odometry` | Ego pose for path start and trimming. |
| `/planning/scenario_planning/scenario` | `autoware_internal_planning_msgs/Scenario` | Optional LaneDriving gating. |
| `/perception/object_recognition/objects` | `autoware_perception_msgs/PredictedObjects` | Lane change safety and obstacle avoidance. |
| `/system/operation_mode/state` | `autoware_adapi_v1_msgs/OperationModeState` | Start planner hold logic. |
| `/planning/scenario_planning/max_velocity` | `autoware_internal_planning_msgs/VelocityLimit` | Path velocity cap. |
| `/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/input/lateral_offset` | `tier4_planning_msgs/LateralOffset` | Side Shift request. |

It publishes:

| ROS1 topic | Message | Purpose |
| --- | --- | --- |
| `/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id` | `autoware_internal_planning_msgs/PathWithLaneId` | Final BPP output path. |
| `/planning/turn_indicators_cmd` | `autoware_vehicle_msgs/TurnIndicatorsCommand` | Turn signal command. |
| `/planning/hazard_lights_cmd` | `autoware_vehicle_msgs/HazardLightsCommand` | Hazard command. |
| `/planning/scenario_planning/modified_goal` | `autoware_planning_msgs/PoseWithUuidStamped` | Parking Goal Planner side output; absent in the non-parking scope. |
| `/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/output/is_reroute_available` | `tier4_planning_msgs/RerouteAvailability` | Goal modification availability. |
| `/planning/cooperate_status/lane_change_left` | `tier4_rtc_msgs/CooperateStatusArray` | Left Lane Change RTC state. |
| `/planning/cooperate_status/lane_change_right` | `tier4_rtc_msgs/CooperateStatusArray` | Right Lane Change RTC state. |
| `/planning/path_candidate/lane_change_*` | `autoware_planning_msgs/Path` | Unapproved Lane Change proposal. |
| `/planning/path_reference/lane_change_*` | `autoware_planning_msgs/Path` | Root path retained while waiting. |
| `/planning/path_candidate/static_obstacle_avoidance` | `autoware_planning_msgs/Path` | Static avoidance or return-shift proposal. |
| `/planning/path_reference/static_obstacle_avoidance` | `autoware_planning_msgs/Path` | Static avoidance root path. |
| `/planning/cooperate_status/static_obstacle_avoidance_left` | `tier4_rtc_msgs/CooperateStatusArray` | Left avoidance RTC state. |
| `/planning/cooperate_status/static_obstacle_avoidance_right` | `tier4_rtc_msgs/CooperateStatusArray` | Right avoidance/return RTC state. |
| `/planning/cooperate_status/start_planner` | `tier4_rtc_msgs/CooperateStatusArray` | Forward pull-out RTC state. |
| `/planning/path_candidate/start_planner` | `autoware_planning_msgs/Path` | Start Planner shift proposal. |
| `/planning/path_candidate/side_shift` | `autoware_planning_msgs/Path` | Side Shift proposal. |
| `/planning/debug/bpp/*` | `nav_msgs/Path`, debug messages | Stage-level debug paths and module statuses. |

## Map Centerline

Run with a Lanelet2 OSM file when available:

```bash
roslaunch behavior_path_planner behavior_path_planner.launch \
  map_file:=/path/to/lanelet2_map.osm \
  projector_file:=/path/to/map_projector_info.yaml
```

If no map is passed, lane following falls back to a straight interpolated path from route start/ego pose to goal.

## Current Port Boundary

This is an Autoware-shaped ROS1 implementation, not a direct source-code clone of Autoware Universe. It keeps the same major BPP module order and topic surface, while using ROS1-compatible custom messages and lightweight Python implementations for module behavior.

Normal lane change is route-driven: the planner selects the lane containing
the ego as the root lane, reads the route section's preferred primitive as the
target lane, and builds the transition from both Lanelet2 centerlines. See
`docs/bpp/LANE_CHANGE_BPP.md` for the validated scope and remaining gaps.

Lane Change execution is gated by the shared ROS1 RTC implementation. See
`docs/bpp/RTC_INTERFACE_ROS1.md` for Auto/Manual behavior and service commands.

Static Obstacle Avoidance now has stateful object filtering, Lanelet2 bound
expansion, margin and jerk constrained shift lines, safety hysteresis, stop
fallback, directional RTC, and Planning Simulator differential baselines. See
`docs/bpp/STATIC_OBSTACLE_AVOIDANCE_BPP.md` for the tested scope and remaining
differences from the Autoware C++ implementation.

Dynamic Obstacle Avoidance, forward Start Planner, and Side Shift use shared
standalone/integrated engines and have K-City differential baselines. See
`DYNAMIC_OBSTACLE_AVOIDANCE_BPP.md`, `START_PLANNER_BPP.md`, and
`SIDE_SHIFT_BPP.md`.

Goal Planner remains dormant for the competition's non-parking route, matching
Autoware's empty Goal Planner candidate/reference output in that scenario. The
fixed route terminal and zero velocity remain part of Lane Following. See
`GOAL_PLANNER_BPP.md`.
