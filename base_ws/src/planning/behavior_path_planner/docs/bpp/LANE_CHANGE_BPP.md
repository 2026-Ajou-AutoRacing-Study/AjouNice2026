# Lane Change BPP ROS1 Port

## Behavior

The normal lane-change path is generated from the mission route and Lanelet2
map instead of a fixed lane-width offset:

1. Select the route primitive closest to the ego as the source lane.
2. Select the route section's `preferred_primitive` as the target lane.
3. Keep the source centerline through the preparation distance.
4. Blend source and target centerlines through the lane-change distance.
5. Continue along the target lane and assign the target lane ID.

The module publishes `ENABLE_LEFT` or `ENABLE_RIGHT` from the signed map
geometry. The older `LateralOffset` request remains available when no
route-driven change is required.

## RTC Execution Gate

Lane Change now uses the shared ROS1 RTC implementation. It always publishes
the proposed candidate and reference paths, then selects the final output from
the RTC mode, safety status, and command. Left and right requests use separate
status topics and services, matching the Autoware naming surface.

See `RTC_INTERFACE_ROS1.md` for topics, services, state transitions, launch
arguments, and command examples.

## K-City Baseline

The differential scenario uses one left lane change:

| Item | Value |
| --- | --- |
| Source lane | `200806` |
| Target/preferred lane | `200759` |
| Route primitives | `[200760, 200759, 200806]` |
| Objects | Empty |
| ROS2 RTC state | Safe, waiting for execution |

The same ROS2 `LaneletRoute`, odometry, map, and empty object state are replayed
into ROS1. The comparator checks route equality, lane-following root geometry,
lane-change candidate geometry and length, start/end positions, transition
distances, target offset/lane ID, velocity surface, and turn signal.

Current result:

```text
Candidate geometry RMSE:       0.238 m
Candidate geometry maximum:    1.177 m
Candidate length error:        1.019 m
Candidate start error:         0.002 m
Candidate end error:           1.177 m
Target lateral offset error:   0.000 m
Nominal start distance error:  0.100 m
Nominal finish distance error: 0.297 m
Target lane ID:                exact match
Turn indicator:                ENABLE_LEFT
Verdict:                       PASS
```

The RViz overlay is launched with `bpp_lane_change_comparison.launch`. ROS2 is
cyan, ROS1 is magenta dashed, ROS2/ROS1 root paths are white/amber, and K-City
lanelet boundaries are gray. The full source lanelet area and centerline are
translucent blue, the full target lanelet area and centerline are translucent
green, and the route goal is shown in yellow. All source/target/goal markers
include their lanelet IDs as RViz text labels.

## Blocked K-City Baseline

A second differential scenario places one stationary `CAR` on target lanelet
`200759`, approximately 20 m ahead of the ego. The complete ROS2
`PredictedObjects` input is serialized and replayed into ROS1 with the same
route and odometry.

The blocked-case comparator verifies that both planners reject execution while
still producing a lane-change proposal and `ENABLE_LEFT`. It compares the
executable final paths rather than treating the rejected ROS2 candidate as an
output path.

```text
Blocking object input:         exact match
ROS2 RTC / ROS1 status:        unsafe / blocked
Final path RMSE:               0.063 m
Final path maximum error:      1.102 m
Final path length error:       1.097 m
Proposal/final max separation: 27.402 m
Final lane IDs:                [200806] / [200806]
Turn indicator:                ENABLE_LEFT / ENABLE_LEFT
Verdict:                       PASS
```

The RViz overlay is launched with
`bpp_lane_change_blocked_comparison.launch`: cyan and magenta show the ROS2
and ROS1 final paths, orange shows the rejected ROS2 proposal, and the red box
shows the blocking object and its predicted path.

## Validation Boundary

These results validate one obstacle-free left lane change and one stationary
target-lane object blocking the same left lane change. They do not validate
right lane change, moving or rear-approaching objects, full predicted-path
collision checks, moving-ego abort trajectories, multiple consecutive lane
changes, or multi-module arbitration. RTC Auto/Manual mode, typed status,
ACTIVATE/DEACTIVATE, and the basic ABORTING transition were tested separately.
The comparison uses 2D driving-plane geometry because the three K-City
parallel lanelets contain an approximately one-meter elevation discontinuity.
