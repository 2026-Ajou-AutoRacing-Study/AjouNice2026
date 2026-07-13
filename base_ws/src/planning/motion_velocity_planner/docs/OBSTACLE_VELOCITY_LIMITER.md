# Obstacle Velocity Limiter

Limits velocity along a reference path based on nearby obstacles.

Inputs (launch defaults):
- `/behavior_path_planner/output/path` : `autoware_internal_planning_msgs/PathWithLaneId`
- `/perception/objects` : `autoware_perception_msgs/PredictedObjects`
- `/odometry` : `nav_msgs/Odometry`
- `/planning/scenario_planning/max_velocity` : `autoware_internal_planning_msgs/VelocityLimit`

Output (launch default):
- `/motion_velocity_planner/obstacle_velocity_limiter/output/path` : `autoware_internal_planning_msgs/PathWithLaneId`

Parameters:
- `~min_ttc` : TTC threshold for limiting speed (default 1.0 s)
- `~distance_buffer` : additional longitudinal obstacle buffer (default 0.0 m)
- `~min_adjusted_velocity_mps` : lowest non-zero velocity requested by the TTC rule (default 2.5 m/s)
- `~max_deceleration` : maximum deceleration imposed on the resulting velocity profile (default 2.0 m/s²)
- `~max_lateral_distance` : objects farther from the path than this are ignored (default 1.5 m)

Behavior:
- For each path point, the nearest predicted object distance is computed.
- If the TTC is less than or equal to `min_ttc`, the point velocity is limited to `distance / min_ttc` (not below `min_adjusted_velocity_mps`).
- At and beyond an obstacle's projected path position, the point velocity is set to 0.
- A backward pass limits deceleration between adjacent path points to `max_deceleration`.
- A global `VelocityLimit.max_velocity` is enforced when provided.
