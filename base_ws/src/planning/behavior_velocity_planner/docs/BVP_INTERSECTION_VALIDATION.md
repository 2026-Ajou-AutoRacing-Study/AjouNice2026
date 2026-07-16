# BVP Intersection ROS 1 Port and Validation

## Implemented behavior

The ROS 1 Intersection scene module follows the Autoware BVP execution shape:

- activates on path lanelets tagged with `turn_direction=straight|left|right`;
- derives conflicting road lanelets and their preceding topology from Lanelet2;
- reads `intersection_area` polygons and `right_of_way` regulatory elements;
- places the default stop point before the first vehicle-footprint conflict;
- compares object predicted-path time intervals with ego TTC;
- detects a stopped vehicle on the ego exit path (`StuckStop`);
- applies the turn-dependent yield-stuck rule (`YieldStuckStop`);
- distinguishes a temporal collision from an object that clears early;
- handles traffic-light priority, collision hold, and the pass-judge line;
- handles occupancy-grid occlusion first-wait and peeking states;
- publishes Intersection StopReason, PlanningFactor/SafetyFactor, status, and
  RViz markers.

It uses the existing ROS 1 Autoware-compatible messages. No new message package
was needed: `PathWithLaneId`, `Path`, `PredictedObjects`, `Odometry`,
`TrafficLightGroupArray`, `OccupancyGrid`, StopReason, and PlanningFactor are
already present in the workspace.

This is an operational ROS 1 port, not a line-by-line C++ transplant. The
validated stop/go decisions and output profiles match the Planning Simulator
cases below. Autoware's separate Intersection/Intersection-Occlusion RTC
channels, every diagnostic marker, full wrong-direction handling, and all
production edge cases are not yet reproduced one-for-one.

Official references:

- <https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module/>
- <https://github.com/autowarefoundation/autoware_universe/tree/0.51.0/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module>

## Map requirement

The production `lanelet2_map_v44.osm` currently has no `turn_direction` tags
and no `intersection_area` polygon. Therefore neither the official module nor
the ROS 1 port can identify an Intersection module target from v44 alone.

Validation uses local copies generated from v44:

```text
local_autoware_planning_sim/data/maps/kcity_intersection_validation_map/
local_autoware_planning_sim/data/maps/kcity_intersection_left_validation_map/
```

The copies tag lanelet `200208`, its conflicting approaches, and a validation
`intersection_area`. The production map is not modified. Before deployment,
the map owner must add correct turn direction, intersection area, right-of-way,
and traffic-light/stop-line relations for each competition-route intersection.

## Differential scenarios

All scenarios use the same route `200335 -> 200208 -> 200166` and the same
captured ROS 2 input path. ROS 2 output from Autoware Planning Simulator is the
reference; its path, odometry, objects, predictions, and occupancy grid are
replayed into ROS 1.

| Scenario | Input condition | ROS 2 decision | ROS 1 decision | Result |
|---|---|---|---|---|
| `clear` | no target object | Safe | Safe | PASS |
| `collision` | 2.0 m/s car crosses during ego TTC | NonOccludedCollisionStop | NonOccludedCollisionStop | PASS |
| `timing_clear` | 8.0 m/s car clears before ego arrival | Safe | Safe | PASS |
| `stuck` | stopped vehicle blocks ego exit | StuckStop | StuckStop | PASS |
| `yield_stuck_straight` | stopped car near conflict, straight rule disabled | Safe | Safe | PASS |
| `yield_stuck_left` | same car, target tagged left | YieldStuckStop | YieldStuckStop | PASS |
| `occlusion` | intersection occupancy window is unobserved | FirstWaitBeforeOcclusion | FirstWaitBeforeOcclusion | PASS |

For every stopping scenario:

- ROS 2 stop station: `19.300 m`;
- ROS 1 stop station: `19.410 m`;
- stop station error: `0.110 m`;
- output geometry RMSE/max: `0.000/0.000 m`;
- velocity RMSE/max error: `0.115/1.250 m/s`.

The maximum velocity error occurs at one sample next to the abrupt zero-speed
boundary. The path geometry is identical and the stop-position difference is
11 cm. Safe scenarios have zero geometry and velocity-profile error.

Unit tests additionally cover pedestrian filtering, pass-judge behavior, and
the `FirstWaitBeforeOcclusion -> PeekingTowardOcclusion` stopped-time state
transition.

## RViz

Run any captured scenario by changing one argument:

```bash
cd ~/AJOUNICE2026/repo/base_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch behavior_velocity_planner bvp_intersection_comparison.launch \
  scenario:=collision
```

Available values are `clear`, `collision`, `timing_clear`, `stuck`,
`yield_stuck_straight`, `yield_stuck_left`, and `occlusion`. The left-turn
yield-stuck case uses the left validation map:

```bash
roslaunch behavior_velocity_planner bvp_intersection_comparison.launch \
  scenario:=yield_stuck_left \
  map_file:=/home/vic/AJOUNICE2026/local_autoware_planning_sim/data/maps/kcity_intersection_left_validation_map/lanelet2_map.osm
```

RViz legend:

- cyan outline: ROS 2 Planning Simulator output;
- magenta outline: ROS 1 port output;
- inner red/yellow/cyan line: zero/intermediate/maximum path speed;
- green polygon: target intersection lanelet `200208`;
- orange polygons/centerlines: conflicting lanelets;
- yellow polygon: `intersection_area`;
- red box and orange line: scenario object and predicted path;
- purple hatched rectangle: unobserved occupancy window;
- cyan/magenta spheres: each implementation's module stop point.

The two speed paths are shifted laterally by `+/-1.2 m` for readability only.
Numerical comparisons use the unshifted paths.
