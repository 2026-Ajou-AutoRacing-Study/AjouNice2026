# Lane Following BPP MVP

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
| `~input/scenario` | `autoware_internal_planning_msgs/Scenario` | Optional LaneDriving gate |
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
and resampled. If a raw OSM payload is published through `LaneletMapBin`, it is
also accepted. If no map is available or no route lane ID matches the map, the
node falls back to a straight route-shaped path from ego odometry or route start
pose to the route goal pose.

The final path point is marked `is_final=true`, and its velocity is set to zero
when `stop_at_goal=true`.

## Run

```bash
cd base_ws
catkin_make
source devel/setup.bash
roslaunch behavior_path_planner lane_following_bpp.launch
```

The top-level launch file is a thin ROS 1 wrapper. The BPP-specific launch body
lives in `launch/lane_following_bpp_impl.launch`.

## Next Step

Replace the OSM-file bridge with native Autoware `LaneletMapBin` binary parsing
or an explicit ROS 1 map adapter when the upstream binary map publisher is
ported.
