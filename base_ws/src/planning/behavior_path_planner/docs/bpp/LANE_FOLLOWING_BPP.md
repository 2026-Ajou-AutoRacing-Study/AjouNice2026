# Lane Following BPP ROS1 Port

This node is the first ROS 1 implementation step for the Autoware Behavior Path
Planner flow. It implements only the lane-following surface of BPP and keeps the
Autoware-like topic contract alive with ROS1 Autoware-compatible messages.

## Node

```text
behavior_path_planner/scripts/lane_following_bpp.py
```

## Inputs

| Topic | Type | Purpose |
| --- | --- | --- |
| `~input/route` | `autoware_planning_msgs/LaneletRoute` | Mission planner route |
| `~input/vector_map` | `autoware_map_msgs/LaneletMapBin` | Optional raw OSM payload; binary Autoware payload is stored only |
| `~input/odometry` | `nav_msgs/Odometry` | Current ego pose used as path start |
| `~input/scenario` | `autoware_internal_planning_msgs/Scenario` | LaneDriving gate |
| `/planning/scenario_planning/max_velocity` | `autoware_internal_planning_msgs/VelocityLimit` | Optional velocity cap |

## Outputs

| Topic | Type | Purpose |
| --- | --- | --- |
| `~output/path` | `autoware_internal_planning_msgs/PathWithLaneId` | BPP output path for BVP input |
| `~output/turn_indicators_cmd` | `autoware_vehicle_msgs/TurnIndicatorsCommand` | Default no-command output |
| `~output/hazard_lights_cmd` | `autoware_vehicle_msgs/HazardLightsCommand` | Default no-command output |
| `~output/modified_goal` | `autoware_planning_msgs/PoseWithUuidStamped` | Current route goal passthrough |
| `~output/is_reroute_available` | `tier4_planning_msgs/RerouteAvailability` | MVP always publishes available |
| `~debug/path` | `nav_msgs/Path` | RViz-friendly debug path |

## Current Behavior

The node first tries to generate the reference path from map centerlines. If
`~map_file` points to a Lanelet2 `.osm` file, route lane IDs are matched to
lanelet relation IDs and the centerlines are stitched, trimmed by ego/goal pose,
and resampled. MGRS, UTM, and LocalCartesianUTM projections are supported.

The path includes Autoware's backward path margin, connects to the exact goal
pose, assigns lane IDs per path point, and publishes the stitched left and right
drivable-area bounds. If no map is available or no route lane ID matches the
map, the node falls back to a straight route-shaped path.

The goal velocity is set to zero when `stop_at_goal=true`. Lane Following keeps
`is_final=false`, matching the captured Autoware BPP output.

## Run

```bash
cd base_ws
catkin_make
source devel/setup.bash
roslaunch behavior_path_planner lane_following_bpp.launch \
  map_file:=/path/to/lanelet2_map.osm \
  map_projector_type:=MGRS
```

The top-level launch file is a thin ROS 1 wrapper. The BPP-specific launch body
lives in `launch/lane_following_bpp_impl.launch`.

## Differential Validation

The K-City validation replays the exact ROS2 `LaneletRoute`, ego odometry, and
LaneDriving scenario into ROS1. It compares path geometry by point-to-segment
distance so different sampling phases do not produce false failures.

The RViz overlay launch renders ROS2 as a cyan solid line and ROS1 as a magenta
dashed line:

```bash
roslaunch behavior_path_planner bpp_lane_following_comparison.launch \
  reference_json:=/path/to/reference.json \
  candidate_json:=/path/to/ros1.json \
  report_json:=/path/to/report.json
```

Native binary `LaneletMapBin` decoding remains outside this ROS1 package; real
maps are loaded from the corresponding Lanelet2 OSM file.
