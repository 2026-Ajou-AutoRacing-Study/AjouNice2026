# Static Obstacle Avoidance BPP ROS1 Port

## Implemented Behavior

The ROS1 port now uses one shared stateful engine from both
`static_obstacle_avoidance_bpp.py` and the integrated
`behavior_path_planner.py`. The engine implements the main Autoware static
avoidance flow:

1. Track `PredictedObjects`, including stopped time and short lost-object
   compensation.
2. Filter by object class, speed, moving time, longitudinal detection range,
   lateral corridor, and parked/ambiguous-vehicle policy.
3. Project the object envelope onto the reference path.
4. Read current and adjacent Lanelet2 bounds and calculate available shift
   space.
5. Apply class-specific soft/hard margins and best-effort clipping.
6. Generate and merge avoidance and return shift lines.
7. Check jerk, path validity, object clearance, and safety hysteresis.
8. Insert a stop profile when a valid avoidance path cannot be executed.
9. Gate left avoidance and right return independently through RTC.

The default parameters mirror the values used by the current Autoware
Planning Simulator launcher, including a `0.5 m` car soft margin, `0.2 m`
parked hard margin, `0.2 m/s^3` minimum lateral jerk, `8.33 m/s` nominal
avoidance speed, and `per_shift_line` approval.

## ROS Interface

| Direction | Topic or service | Type |
| --- | --- | --- |
| Input | `/planning/bpp/lane_following/path` | `autoware_internal_planning_msgs/PathWithLaneId` |
| Input | `/perception/object_recognition/objects` | `autoware_perception_msgs/PredictedObjects` |
| Input | `/localization/kinematic_state` | `nav_msgs/Odometry` |
| Output | `/planning/bpp/static_obstacle_avoidance/path` | `autoware_internal_planning_msgs/PathWithLaneId` |
| Candidate | `/planning/path_candidate/static_obstacle_avoidance` | `autoware_planning_msgs/Path` |
| Reference | `/planning/path_reference/static_obstacle_avoidance` | `autoware_planning_msgs/Path` |
| RTC status | `/planning/cooperate_status/static_obstacle_avoidance_left` | `tier4_rtc_msgs/CooperateStatusArray` |
| RTC status | `/planning/cooperate_status/static_obstacle_avoidance_right` | `tier4_rtc_msgs/CooperateStatusArray` |
| RTC command | `/planning/cooperate_commands/static_obstacle_avoidance_left` | `tier4_rtc_msgs/CooperateCommands` |
| RTC command | `/planning/cooperate_commands/static_obstacle_avoidance_right` | `tier4_rtc_msgs/CooperateCommands` |
| RTC auto mode | `/planning/enable_auto_mode/static_obstacle_avoidance_left` | `tier4_rtc_msgs/AutoMode` |
| RTC auto mode | `/planning/enable_auto_mode/static_obstacle_avoidance_right` | `tier4_rtc_msgs/AutoMode` |

The integrated node publishes its final path on Autoware's BPP output topic:

```text
/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id
```

## Differential Scenario

The K-City baseline replays exactly the same neutral inputs into ROS2
Planning Simulator and ROS1:

| Item | Value |
| --- | --- |
| Ego lane | `200806` |
| Route primitives | `[200760, 200759, 200806]` |
| Object | Stationary `CAR`, `4.5 x 2.0 x 1.6 m` |
| Object location | About `45 m` ahead and `1.8 m` right of centerline |
| Planned response | About `1.0 m` left shift |
| Scenario | `LaneDriving` |

Path point indices are not compared directly. ROS2 emits a dense candidate
near `0.3 m` spacing while this ROS1 chain retains the `2.0 m` lane-following
spacing. Both paths are projected onto their own root centerline and sampled
at common one-meter stations.

## Validation Result

Manual RTC keeps the avoidance candidate visible while preserving the root
path as the executable output:

```text
Input route/ego/object:            exact match
Root path RMSE / maximum:          0.006 / 0.018 m
Candidate lateral RMSE / maximum:  0.154 / 0.383 m
Peak shift ROS2 / ROS1:            1.031 / 1.000 m
RTC start error / finish error:    0.739 / 0.061 m
RTC state:                         safe, manual, WAITING
Executable output:                 unchanged root path
Turn indicator:                    DISABLE / DISABLE
Verdict:                           PASS
```

Automatic RTC applies the first shift while holding the separately managed
return shift:

```text
Return-candidate RMSE / maximum:    0.156 / 0.311 m
Executable-path RMSE / maximum:    0.205 / 0.453 m
Peak output shift ROS2 / ROS1:     1.133 / 1.000 m
Avoidance RTC:                     safe, auto, RUNNING
Return RTC:                        unsafe, auto, WAITING
Return RTC interval ROS1:          48.461 to 93.683 m
Turn indicator:                    DISABLE / DISABLE
Standalone verdict:               PASS
Integrated BPP verdict:            PASS
```

The turn indicator follows Autoware's swerving rule. Even though the shift is
larger than `0.3 m`, it is disabled when the remaining clearance to the lane
bound is below `1.3 m`.

## RViz Comparison

Use `validation/launch/bpp_static_avoidance_comparison.launch` with the captured reference,
ROS1 result, and report. The automatic view shows:

- cyan: ROS2 executable path;
- magenta dashed: ROS1 executable path;
- orange: ROS2 return-shift candidate;
- white and amber: ROS2 and ROS1 root paths;
- red box: the stationary blocking object; and
- gray: K-City Lanelet2 boundaries.

The exact commands and local capture locations are documented in
`local_autoware_planning_sim/tools/BPP_STATIC_OBSTACLE_AVOIDANCE_VALIDATION.md`.

## Remaining Difference From Autoware

This is a high-fidelity ROS1 behavior port, not a source-identical build of
the Autoware Universe C++ plugin. The validated behavior, message surface,
parameter policy, RTC states, and path shape agree for the K-City baseline.
The following still require additional work before claiming full equivalence:

- the exact C++ `PathShifter` spline and dense resampling implementation;
- complete Lanelet2 regulatory-element and traffic-rule handling;
- Autoware's full multi-hypothesis predicted-path RSS safety checker;
- every edge policy for red signals, crosswalks, parking violations, and
  adjacent moving traffic;
- all planning-factor, marker, and object-of-interest debug outputs; and
- scenario coverage for right avoidance, opposing traffic, multiple objects,
  moving ego progression, abort, and lost perception.
