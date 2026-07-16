# Motion Velocity Planner ROS 1 Port

The production entry point is `scripts/motion_velocity_planner.py`. It directly
receives the `autoware_planning_msgs/Trajectory` produced by Path Optimizer and
publishes another `Trajectory` after applying velocity constraints.

| Direction | Topic | Message |
|---|---|---|
| Subscribe | `/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory` | `autoware_planning_msgs/Trajectory` |
| Subscribe | `/perception/object_recognition/objects` | `autoware_perception_msgs/PredictedObjects` |
| Subscribe | `/localization/kinematic_state` | `nav_msgs/Odometry` |
| Subscribe | `/planning/scenario_planning/max_velocity` | `autoware_internal_planning_msgs/VelocityLimit` |
| Publish | `/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/trajectory` | `autoware_planning_msgs/Trajectory` |

The main node does not convert the trajectory back to `PathWithLaneId`. Lane IDs
are required by BPP and BVP, but the post-optimizer motion-planning interface is
a sampled `Trajectory` without lane IDs.

The older individual Python module scripts remain useful as isolated algorithm
experiments. They are not the nodes used by the integrated production launch.
See `docs/validation/README.md` for current commands, results, and limitations.
