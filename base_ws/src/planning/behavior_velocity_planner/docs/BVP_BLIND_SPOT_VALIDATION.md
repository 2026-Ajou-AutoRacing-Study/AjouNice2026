# BVP Blind Spot ROS 1 Port and Validation

## Implemented behavior

The ROS 1 Blind Spot scene module follows the Autoware module contract:

- activates only on route lanelets tagged `turn_direction=left|right`;
- constructs the blind-side attention strip and ego turn conflict strip;
- targets pedestrian, bicycle, and motorcycle objects by default;
- optionally enables car, truck, bus, and trailer detection through parameters;
- requires the object's current pose to be in the attention area;
- resamples predicted paths at `0.1 s` and compares object/ego passage times;
- applies the official `2.0 s` entry and `1.0 s` exit TTC margins;
- holds STOP for `0.5 s` before returning to GO;
- applies critical braking and over-pass judgment instead of commanding an
  abrupt, unreachable stop;
- inserts the stop point on the same `0.2 m` interpolated-path grid used by
  the official module;
- publishes Blind Spot StopReason, PlanningFactor/SafetyFactor, status, and
  attention/conflict/object debug markers.

No new message package was required. The module reuses the workspace's ROS 1
ports of `PathWithLaneId`, `Path`, `PredictedObjects`, `Odometry`,
StopReasonArray, and PlanningFactorArray.

This is a behavior-compatible ROS 1 implementation, not a line-for-line C++
transplant. Autoware's routing-graph artificial-lanelet construction and every
diagnostic/RTC surface are not reproduced one-for-one. The validated stop/go,
stop position, path geometry, and velocity outputs match the official Planning
Simulator cases below.

Official references:

- <https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module/>
- <https://github.com/autowarefoundation/autoware_universe/tree/0.51.0/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module>

## Map requirement

Blind Spot does not activate from geometry alone. The turning lanelet must
carry the correct `turn_direction=left` or `turn_direction=right` tag. The
production `lanelet2_map_v44.osm` currently has no turn-direction tags.

Validation therefore uses isolated copies of v44:

```text
local_autoware_planning_sim/data/maps/kcity_blind_spot_left_validation_map/
local_autoware_planning_sim/data/maps/kcity_blind_spot_right_validation_map/
```

The left copy tags the actual left-turn lanelet `200452`; its route is
`200335 -> 200452 -> 200201`. The right copy tags the actual right-turn lanelet
`200436`; its route is `200335 -> 200436 -> 200221`. The production map is not
modified.

An earlier synthetic test tagged straight lanelet `200208` as left. Official
Autoware correctly returned an internal geometry error because a straight
lanelet cannot cross the generated Blind Spot conflict boundaries. That case
was discarded rather than treated as a port failure.

## Differential scenarios

ROS 2 Autoware Planning Simulator was run with Intersection, Traffic Light,
Crosswalk, and Walkway disabled so only Blind Spot could modify the path. Each
captured ROS 2 path, odometry, object, classification, shape, and predicted
path was then replayed unchanged into ROS 1.

| Scenario | Input condition | ROS 2 | ROS 1 | Result |
|---|---|---|---|---|
| `clear_left` | no object | Safe | Safe | PASS |
| `bicycle_collision_left` | bicycle reaches left-turn conflict during ego TTC | Unsafe | Unsafe | PASS |
| `pedestrian_collision_left` | pedestrian reaches the same conflict | Unsafe | Unsafe | PASS |
| `timing_clear_left` | slow bicycle arrives after the ego interval | Safe | Safe | PASS |
| `parallel_conflict_left` | apparent parallel VRU prediction still enters conflict | Unsafe | Unsafe | PASS |
| `car_ignored_left` | car follows the collision geometry; car detection disabled | Safe | Safe | PASS |
| `outside_attention_left` | bicycle starts outside the attention strip | Safe | Safe | PASS |
| `bicycle_collision_right` | mirrored right-side bicycle conflict | Unsafe | Unsafe | PASS |

For all four Unsafe cases:

- ROS 2 stop station: `9.000 m`;
- ROS 1 stop station: `9.000 m`;
- stop-station error: `0.000 m`;
- output geometry RMSE/max: `0.000/0.000 m`;
- velocity RMSE/max error: `0.000/0.000 m/s`.

All four Safe cases have equal decisions, path geometry, and velocity
profiles. Eight unit tests separately cover left/right collision, object type
filtering, attention-area filtering, TTC separation, STOP-to-GO debounce, and
over-pass judgment.

## RViz

The default command opens the left bicycle collision comparison:

```bash
cd ~/AJOUNICE2026/repo/base_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch behavior_velocity_planner bvp_blind_spot_comparison.launch \
  scenario:=bicycle_collision_left
```

Change `scenario` to any left case in the table. For the right-turn case:

```bash
roslaunch behavior_velocity_planner bvp_blind_spot_comparison.launch \
  scenario:=bicycle_collision_right \
  map_file:=/home/vic/AJOUNICE2026/local_autoware_planning_sim/data/maps/kcity_blind_spot_right_validation_map/lanelet2_map.osm
```

RViz legend:

- cyan outline: ROS 2 Planning Simulator output;
- magenta outline: ROS 1 port output;
- inner red/yellow/cyan line: zero/intermediate/maximum path speed;
- green polygon/line: target turning lanelet and centerline;
- translucent blue: blind-side attention area;
- translucent orange: ego turn conflict area;
- red box and orange line: scenario object and predicted path;
- white arrow: ego pose;
- cyan/magenta spheres: each implementation's stop point.

The two speed paths are shifted laterally by `+/-1.2 m` only for readability.
All numerical comparisons use the original unshifted paths.
