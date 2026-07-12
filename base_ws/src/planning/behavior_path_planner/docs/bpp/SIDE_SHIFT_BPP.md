# Side Shift ROS 1 Port

## Implemented Behavior

- consumes the Autoware `tier4_planning_msgs/LateralOffset` request
- keeps only the latest request and rate-limits updates
- tracks `BEFORE_SHIFT`, `SHIFTING`, and `AFTER_SHIFT`
- calculates shift length from lateral jerk and minimum shifting speed
- freezes the active target while shifting and defers a new request
- clamps the requested offset to the drivable bounds and vehicle margin

The integrated input topic matches Autoware:

```text
/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/input/lateral_offset
```

The competition does not provide this external input, so the production launch
sets `enable_side_shift=false` by default. The implementation is retained only
for regression tests and future remote-offset use.

## K-City Scenario

1. Ego is stopped on the route centerline.
2. A `+0.80 m` left Side Shift request is published.
3. The shift starts after the 5 m preparation distance.
4. The path reaches and holds the requested offset inside the drivable bound.
5. The module explicitly disables the turn indicators, matching the ROS 2 output.

Result: candidate and applied-output profile RMSE `0.018 m`, maximum error `0.065 m`.

## RViz

```bash
roslaunch behavior_path_planner bpp_side_shift_comparison.launch \
  reference_json:=/home/vic/AJOUNICE2026/local_autoware_planning_sim/bags/bpp_side_shift/kcity_left_080.reference.json \
  candidate_json:=/home/vic/AJOUNICE2026/local_autoware_planning_sim/results/bpp_side_shift/kcity_left_080.ros1.json \
  report_json:=/home/vic/AJOUNICE2026/local_autoware_planning_sim/results/bpp_side_shift/kcity_left_080.report.json
```
