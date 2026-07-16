# K-City Interactive Mission Planner + BPP + BVP

This launch displays the K-City Lanelet2 map and runs the ROS1 Mission Planner
Behavior Path Planner, and Behavior Velocity Planner together.

```bash
cd /home/vic/AJOUNICE2026/repo/base_ws
source /opt/ros/noetic/setup.bash
source devel_optimizer/setup.bash

roslaunch bringup kcity_mission_bpp_bvp_rviz.launch
```

In RViz:

1. Select `2D Pose Estimate` and click-drag on a K-City lane to set the ego pose.
2. Select `2D Nav Goal` and click-drag on a reachable lane to set the goal.
3. The Mission Route, magenta BPP Path, and cyan BVP Path are published automatically.

`behavior_path_planner: waiting for LaneletRoute` is expected before both inputs
are set. It should disappear after Mission Planner accepts the goal and publishes
the route.

The interactive launch uses a `1500 m` forward path only so a static RViz ego
pose can display the complete route. The normal runtime default remains the
Autoware-like rolling `300 m` horizon.

Topics:

| Purpose | Topic |
| --- | --- |
| RViz start input | `/initialpose` |
| RViz goal input | `/move_base_simple/goal` |
| Mission route | `/planning/mission_planning/route` |
| Mission route marker | `/planning/mission_planning/mission_planner/debug/route_marker` |
| BPP output | `/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id` |
| BPP RViz path | `/planning/debug/bpp/path` |
| BVP output | `/planning/scenario_planning/lane_driving/behavior_planning/path` |
| BVP stop-line markers | `/planning/debug/behavior_velocity_planner/markers` |

## Full trajectory preview

To run the full lane-driving planning chain and display its final trajectory:

```bash
roslaunch bringup kcity_interactive_trajectory_rviz.launch
```

Set the start with `2D Pose Estimate`, then set the destination with
`2D Nav Goal`. The final trajectory is colored by velocity: green is cruise,
yellow/orange is reduced speed, and red is stop.

This RViz-only launch advances a virtual ego along the planner output. BPP can
therefore replan consecutive lane changes and the visualizer accumulates the
traversed poses into one start-to-goal preview. The planner's final Trajectory
topic remains the unmodified rolling output; only the two debug visualization
topics contain the accumulated preview.

If a goal requires several lane changes but is too close to complete them using
the configured prepare and lane-change distances, the preview stops at the last
safe reachable point. Move the goal farther forward on that lane; the tool does
not draw a direct line across lanes to hide an infeasible maneuver.

| Purpose | Topic |
| --- | --- |
| Final trajectory | `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/trajectory` |
| RViz trajectory path | `/planning/debug/final_trajectory/path` |
| Velocity-colored markers | `/planning/debug/final_trajectory/markers` |
