# Behavior Path Planner ROS 1 Port

This file keeps its historical name, but the package is no longer only a topic-flow MVP.
The seven lane-driving BPP stages share reusable engines between standalone nodes and the
integrated `behavior_path_planner.py` node. Parking-specific planning is intentionally excluded.

## Pipeline

| Order | Module | ROS 1 behavior | Planning Simulator validation |
| --- | --- | --- | --- |
| 1 | Lane Following | Lanelet2 route centerline and fixed-goal terminal path | PASS |
| 2 | Lane Change | Route-preferred lane change, object safety check, directional RTC | PASS, normal and blocked |
| 3 | Static Obstacle Avoidance | Object filtering, avoidance/return shift, bounds and RTC | PASS |
| 4 | Dynamic Obstacle Avoidance | Direction/TTC filtering and drivable-bound cutting | PASS |
| 5 | Start Planner | Forward shift pull-out, object safety check and RTC | PASS |
| 6 | Goal Planner | Dormant in the non-parking competition scope; preserves fixed-goal path | PASS |
| 7 | Side Shift | Latest-request state machine, jerk-limited shift and bound clamp | PASS |

Goal Planner and Side Shift are preserved but disabled by default in the
competition launch. Goal Planner requires a parking/pull-over mission, and Side
Shift requires an external `LateralOffset` request.

## Shared Contract

Modules after Lane Following use the following ROS 1 path-in/path-out contract:

| Direction | Topic | Type |
| --- | --- | --- |
| Input | `~input/path` | `autoware_internal_planning_msgs/PathWithLaneId` |
| Output | `~output/path` | `autoware_internal_planning_msgs/PathWithLaneId` |
| Reference | `~output/reference_path` | `autoware_planning_msgs/Path` |
| Candidate | `~output/candidate_path` | `autoware_planning_msgs/Path` |
| Debug | `~debug/path` | `nav_msgs/Path` |
| Debug | `~debug/status` | `std_msgs/String` |

The integrated node keeps the Autoware topic names, including:

```text
/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id
/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/input/lateral_offset
/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/output/is_reroute_available
```

## Validation Matrix

The ROS 2 reference and ROS 1 candidate receive the same route, odometry, objects, and module
request. Geometry is compared after projection onto the common reference path, so different
point spacing does not create false failures.

| Module | K-City scenario | Main checks | Result |
| --- | --- | --- | --- |
| Lane Following | Fixed route | lane IDs, geometry, endpoint | PASS |
| Lane Change | `200806 -> 200759` | path profile, RTC, turn signal | PASS |
| Lane Change blocked | Object in target lane | `safe=false`, current-lane hold | PASS |
| Static Avoidance | Parked object on right | candidate/return profile, RTC | PASS |
| Dynamic Avoidance | Oncoming object on left | object classification, TTC, bound cut | PASS |
| Start Planner | Ego 1.2 m right of center | profile RMSE 0.019 m, RTC | PASS |
| Goal Planner | Non-parking fixed goal | dormant module, endpoint/stop, no modified goal | PASS |
| Side Shift | `+0.80 m` request | profile RMSE 0.018 m, command | PASS |

`catkin_make run_tests_behavior_path_planner` also passes 18 engine tests.

## RViz Comparisons

Validation-only launch, RViz, and publisher files are grouped under
`validation/` and are excluded when the package is built with
`CATKIN_ENABLE_TESTING=OFF`. Each module keeps its own comparison launch:

```text
bpp_lane_following_comparison.launch
bpp_lane_change_comparison.launch
bpp_static_avoidance_comparison.launch
bpp_dynamic_avoidance_comparison.launch
bpp_start_planner_comparison.launch
bpp_goal_planner_comparison.launch
bpp_side_shift_comparison.launch
```

The overlays use cyan for the ROS 2 reference, magenta dashes for the ROS 1 result, orange for
an unapproved proposal where applicable, and include Ego, objects, bounds, goal, and PASS/FAIL
metrics.

## Excluded Scope

The competition has no parking mission. The following Autoware behaviors are therefore not
ported:

- Goal Planner pull-over search and parking-path selection
- reverse, geometric, clothoid, and freespace pull-out
- occupancy-grid parking planning

The fixed route still ends at zero velocity through Lane Following. In a non-parking Autoware
scenario, Goal Planner reference/candidate topics are empty and `modified_goal` is not emitted;
the ROS 1 port follows that behavior.
