# BVP Crosswalk and Walkway ROS 1 Port and Validation

## Implemented behavior

### Crosswalk

- parses Lanelet2 `crosswalk` lanelet polygons and linked regulatory stop lines;
- filters pedestrian, bicycle, motorcycle, and unknown objects;
- intersects object predicted paths with the ego attention area;
- calculates Autoware-style TTC and TTV using the 2.0 m/s ego and 1.39 m/s
  object minimum assumed speeds;
- selects `EGO_PASS_FIRST`, `EGO_PASS_LATER`, or `YIELD` with the default
  4.0/13.0 s margins and 0.5 s additional margins;
- places the stop point using a linked stop line when available, otherwise
  `crosswalk entry - 3.5 m - vehicle front offset`;
- supports object-preferred, crosswalk-limit, jerk-aware no-stop, and
  Lanelet2 `safety_slow_down_speed` decisions;
- publishes StopReason, PlanningFactor with object UUID/TTC, and RViz markers.

The optional occupancy-grid occlusion logic, parked-vehicle slowdown,
crosswalk-obstruction prevention, and traffic-light-specific pedestrian
filtering are not included yet. They are separate extensions from the core
predicted-object yield behavior validated here.

### Walkway

- parses Lanelet2 `walkway` lanelet polygons and linked regulatory stop lines;
- always inserts a mandatory stop while in `APPROACH`;
- changes to `STOP` after the vehicle is within 1.0 m and has remained stopped
  for 0.1 s;
- changes to `SURPASSED` on the following stopped cycle and releases the path;
- publishes Walkway StopReason, PlanningFactor detail `walkway_stop`, and RViz
  markers.

No new ROS 1 message package was required. Both modules use the existing
Autoware-compatible `PredictedObjects`, `PathWithLaneId`, `Path`, StopReason,
and PlanningFactor messages.

Official references:

- <https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module/>
- <https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_walkway_module/>
- <https://github.com/autowarefoundation/autoware_universe/tree/0.51.0/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module>
- <https://github.com/autowarefoundation/autoware_universe/tree/0.51.0/planning/behavior_velocity_planner/autoware_behavior_velocity_walkway_module>

## Crosswalk scenario

1. Autoware Planning Simulator loads the K-City v44 map and Ego follows the
   fixed route beginning in lanelet `200332`.
2. The route intersects crosswalk lanelet `201266` at station 21.45 m.
3. A 1.4 m/s pedestrian starts 4.0 m to the side and has a perpendicular
   predicted path through the ego path.
4. All traffic lights on the route are green, isolating the Crosswalk module.
5. ROS 2 reports the pedestrian as unsafe and inserts a stop at 14.159 m.
6. The captured ROS 2 input path, odometry, UUID, classification, velocity, and
   predicted path are replayed unchanged into ROS 1.
7. ROS 1 loads the same v44 map and inserts its stop at the same 14.159 m
   station.

## Walkway scenario

The production K-City map contains 89 crosswalks and no walkway. Validation
therefore uses a local map copy in which only lanelet `201266` is reclassified
from `crosswalk` to `walkway`. The production OSM remains unchanged.

1. Ego follows the same K-City route and intersects walkway `201266`.
2. No pedestrian or obstacle is required.
3. ROS 2 inserts the mandatory stop at 14.165 m with detail `walkway_stop`.
4. The exact input path and odometry are replayed into ROS 1.
5. ROS 1 inserts the stop at the same station.
6. Unit tests separately verify `APPROACH -> STOP -> SURPASSED` and path release.

## Differential results

| Module | ROS 2 stop | ROS 1 stop | Stop error | Geometry RMSE | Velocity RMSE | Result |
|---|---:|---:|---:|---:|---:|---:|
| Crosswalk | 14.159 m | 14.159 m | 0.000 m | 0.000 m | 0.000 m/s | PASS |
| Walkway | 14.165 m | 14.165 m | 0.000 m | 0.000 m | 0.000 m/s | PASS |

The v44 differential comparison reports a maximum geometry error of 0.001 m
and zero velocity-profile error. Lanelet `201266` has identical members and
geometry in the previous K-City map and v44, but both the current ROS 2 capture
and ROS 1 replay explicitly use v44.

Generated reference and result files are local and excluded from Git:

```text
local_autoware_planning_sim/bags/bvp_crosswalk_v44/
local_autoware_planning_sim/bags/bvp_walkway/
local_autoware_planning_sim/results/bvp_crosswalk_v44/
local_autoware_planning_sim/results/bvp_walkway/
```

## RViz

Crosswalk:

```bash
cd ~/AJOUNICE2026/repo/base_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch behavior_velocity_planner bvp_crosswalk_comparison.launch
```

Walkway:

```bash
roslaunch behavior_velocity_planner bvp_walkway_comparison.launch
```

- thick cyan outline: ROS 2 Planning Simulator output;
- thick magenta outline: ROS 1 output;
- inner red/yellow/cyan line: zero/intermediate/maximum path velocity;
- cyan or green translucent polygon: target crosswalk or walkway;
- red cylinder and orange line: pedestrian and predicted path (Crosswalk only);
- cyan/magenta spheres: each implementation's first zero-velocity point.

The two paths are shifted laterally by `+/-1.2 m` only for readability. The
reported numerical comparison uses the original unshifted coordinates.
