# Goal Planner Non-Parking Contract

Autoware Goal Planner is a pull-over and parking module. With a normal lane-driving goal and no
parking request, it remains inactive: its reference and candidate paths are empty, it does not
publish `modified_goal`, and reroute availability remains true.

The ROS 1 port deliberately follows that behavior. It does not invent a parking maneuver or move
the route goal. Lane Following owns the fixed-goal terminal point and zero terminal velocity.
The competition launch therefore sets `enable_goal_planner=false` by default.

## K-City Checks

- Goal Planner does not alter the input `PathWithLaneId`.
- ROS 2 and ROS 1 final-path geometry and endpoint error are `0.000 m`.
- Both terminal velocities are `0 m/s`.
- Neither side emits a non-parking `modified_goal`.
- Both publish reroute availability `true`.

## RViz

```bash
roslaunch behavior_path_planner bpp_goal_planner_comparison.launch \
  reference_json:=/home/vic/AJOUNICE2026/local_autoware_planning_sim/bags/bpp_goal_planner/kcity_fixed_goal.reference.json \
  candidate_json:=/home/vic/AJOUNICE2026/local_autoware_planning_sim/results/bpp_goal_planner/kcity_fixed_goal.ros1.json \
  report_json:=/home/vic/AJOUNICE2026/local_autoware_planning_sim/results/bpp_goal_planner/kcity_fixed_goal.report.json
```

Parking-lot goal search, pull-over candidates, arc paths, and freespace fallback are outside the
competition scope.
