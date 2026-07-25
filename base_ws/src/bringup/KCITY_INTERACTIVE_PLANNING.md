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

The interactive launch uses a fixed RViz ego pose and a `5000 m` forward path so
the complete route can be inspected as one static planning snapshot.

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

The default mode is a static planning snapshot. The ego odometry stays at the
clicked start pose and the visualizer displays the current final Trajectory
without simulated motion or trajectory history accumulation.

In static mode, BPP plans from one fixed ego pose and may show only the first
feasible lane-change stage. It does not draw a direct line across lanes to hide
an infeasible maneuver.

| Purpose | Topic |
| --- | --- |
| Final trajectory | `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/trajectory` |
| RViz trajectory path | `/planning/debug/final_trajectory/path` |
| Velocity-colored markers | `/planning/debug/final_trajectory/markers` |

## Competition course preset

To keep ego fixed and draw the complete final trajectory from the competition
start to the finish, use:

```bash
roslaunch bringup kcity_competition_full_trajectory_rviz.launch
```

This runs Mission Planner, BPP, BVP, path smoother, path optimizer, and MVP for
the complete `full` preset with a `5000 m` planning horizon. It does not require
a second `rosrun` command.

When using the generic interactive launch instead, load the photographed
competition course separately:

```bash
rosrun bringup kcity_competition_route_publisher.py
```

The preset uses K-City v44 lanelet `200889` as the start, `200445` as an outer
turn waypoint, and `200892` as the finish immediately before the start. The
waypoint is required because start and finish alone allow Lanelet2 to select a
shorter internal branch instead of the complete loop.

## Dynamic ego full-lap validation

To move a virtual ego over the complete v44 competition route while checking
that the final trajectory is continuously replanned ahead of the vehicle, run:

```bash
roslaunch bringup kcity_competition_dynamic_ego_rviz.launch
```

The launch drives ego by route arc length and validates every new trajectory.
It fails immediately if a trajectory point appears behind ego, the trajectory
or BPP route section regresses, the forward horizon disappears, or the path
leaves the reference course. RViz shows the moving ego, progress, the complete
course, and the current velocity-colored 200 m trajectory. Its default virtual
ego settings are a 3 m planning step and 5.0 m/s reported speed; both can be
overridden with `step_distance` and `nominal_speed` launch arguments.

For a faster non-visual regression run:

```bash
roslaunch bringup kcity_competition_dynamic_ego_rviz.launch \
  launch_rviz:=false keep_alive:=false step_distance:=10.0
```

The machine-readable result is written to
`/tmp/kcity_dynamic_ego_validation.report.json`.
