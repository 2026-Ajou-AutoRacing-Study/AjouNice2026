# K-City Interactive Mission Planner + BPP + BVP

This launch displays the K-City Lanelet2 map and runs the ROS1 Mission Planner
Behavior Path Planner, and Behavior Velocity Planner together.

```bash
cd /home/vic/AJOUNICE2026/repo/base_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source /home/vic/AJOUNICE2026/local_ros_deps/source_lanelet2.sh

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
