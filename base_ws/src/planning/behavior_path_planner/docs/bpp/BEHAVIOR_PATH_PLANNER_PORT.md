# Behavior Path Planner ROS1 Port

This package now provides two BPP execution styles:

| Launch file | Shape | Use case |
| --- | --- | --- |
| `behavior_path_planner.launch` | One integrated BPP node | Autoware-style runtime path: lane following, lane change, obstacle avoidance, start/goal, and side shift are applied inside one planning cycle. |
| `behavior_path_planner_mvp.launch` | Multiple simple nodes connected by topics | Module-level debugging and step-by-step inspection. |

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

It publishes:

| ROS1 topic | Message | Purpose |
| --- | --- | --- |
| `/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id` | `autoware_internal_planning_msgs/PathWithLaneId` | Final BPP output path. |
| `/planning/turn_indicators_cmd` | `autoware_vehicle_msgs/TurnIndicatorsCommand` | Turn signal command. |
| `/planning/hazard_lights_cmd` | `autoware_vehicle_msgs/HazardLightsCommand` | Hazard command. |
| `/planning/scenario_planning/modified_goal` | `autoware_planning_msgs/PoseWithUuidStamped` | Goal planner side output. |
| `/planning/scenario_planning/is_reroute_available` | `tier4_planning_msgs/RerouteAvailability` | Goal modification availability. |
| `/planning/cooperate_status/lane_change_left` | `tier4_rtc_msgs/CooperateStatusArray` | Left Lane Change RTC state. |
| `/planning/cooperate_status/lane_change_right` | `tier4_rtc_msgs/CooperateStatusArray` | Right Lane Change RTC state. |
| `/planning/path_candidate/lane_change_*` | `autoware_planning_msgs/Path` | Unapproved Lane Change proposal. |
| `/planning/path_reference/lane_change_*` | `autoware_planning_msgs/Path` | Root path retained while waiting. |
| `/planning/path_candidate/static_obstacle_avoidance` | `autoware_planning_msgs/Path` | Static avoidance or return-shift proposal. |
| `/planning/path_reference/static_obstacle_avoidance` | `autoware_planning_msgs/Path` | Static avoidance root path. |
| `/planning/cooperate_status/static_obstacle_avoidance_left` | `tier4_rtc_msgs/CooperateStatusArray` | Left avoidance RTC state. |
| `/planning/cooperate_status/static_obstacle_avoidance_right` | `tier4_rtc_msgs/CooperateStatusArray` | Right avoidance/return RTC state. |
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
