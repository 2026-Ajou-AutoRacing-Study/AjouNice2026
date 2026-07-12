# Mission Planner ROS1 Port

This package keeps the Autoware mission planner route surface available in ROS 1.

## Scope

The node publishes `autoware_planning_msgs/LaneletRoute` to:

```text
/planning/mission_planning/route
```

It also publishes route state and debug markers:

```text
/planning/mission_planning/state
/planning/mission_planning/mission_planner/debug/route_marker
```

## Inputs

| Topic or service | Type | Purpose |
| --- | --- | --- |
| `/localization/kinematic_state` | `nav_msgs/Odometry` | Current ego pose used as route start. |
| `/system/operation_mode/state` | `autoware_adapi_v1_msgs/OperationModeState` | Operation mode input for reroute safety surface compatibility. |
| `/map/vector_map` | `autoware_map_msgs/LaneletMapBin` | Optional OSM payload. Binary Lanelet2 parsing is not implemented. |
| `/planning/scenario_planning/modified_goal` | `autoware_planning_msgs/PoseWithUuidStamped` | Goal update for the active route UUID. |
| `/planning/mission_planning/mission_planner/set_lanelet_route` | `autoware_internal_planning_msgs/SetLaneletRoute` | Autoware-compatible route service. |
| `/planning/mission_planning/mission_planner/set_waypoint_route` | `autoware_internal_planning_msgs/SetWaypointRoute` | Autoware-compatible waypoint route service. |
| `/planning/mission_planning/mission_planner/clear_route` | `autoware_internal_planning_msgs/ClearRoute` | Clear current route. |

## Lanelet2 Routing Port

The default path uses the ROS Noetic Lanelet2 implementation for the same core
operations as Autoware:

- vehicle traffic rules and a Lanelet2 routing graph;
- direction-aware shortest paths between odometry, checkpoints, and goal;
- MGRS, UTM, LocalCartesianUTM, and LocalCartesian projection;
- route sections containing lane-changeable and connected adjacent lanes;
- goal orientation, footprint, and height validation;
- route state transitions, modified goals, reroute distance checks, and stopped
  arrival checks.

Install the runtime dependency once on the ROS1 PC:

```bash
sudo apt update
sudo apt install ros-noetic-lanelet2 ros-noetic-lanelet2-python
```

`route_lane_ids` is now empty by default. Supplying IDs explicitly enables only
the legacy fixed-route test mode and bypasses graph search.

## Remaining Differences

- The ROS1 node loads a Lanelet2 OSM file directly. ROS2 serialized
  `LaneletMapBin` archives are not decoded across distributions.
- Parking-area routing, MRM route selection, and preferred-primitive service
  handling are not part of this package yet.
- The footprint and autonomous reroute safety checks mirror the original
  thresholds but are simpler than Autoware's complete utility implementation.

## Minimal Mission Planner Test

```bash
cd /home/vic/AJOUNICE2026/repo/base_ws
source devel/setup.bash
roslaunch mission_planner mission_planner_minimal_test.launch
```

For a real map, leave `route_lane_ids` empty and pass the projection used by
Autoware. K-City uses MGRS:

```bash
roslaunch mission_planner mission_planner.launch \
  map_file:=/path/to/lanelet2_map.osm \
  map_projector_type:=MGRS
```

RViz topics:

```text
/planning/mission_planning/mission_planner/debug/route_marker
/planning/mission_planning/mission_planner/debug/goal_pose
```

## ROS2 and ROS1 Route Overlay

The reference and candidate JSON files can be rendered together in ROS1 RViz.
The ROS2 route is a cyan solid line, the ROS1 route is a magenta dashed line,
and route-only lanelets are highlighted. Matching sections receive an `OK`
label.

```bash
roslaunch mission_planner mission_route_comparison.launch \
  map_file:=/path/to/lanelet2_map.osm \
  reference_json:=/path/to/reference.json \
  candidate_json:=/path/to/ros1.json \
  report_json:=/path/to/report.json
```
