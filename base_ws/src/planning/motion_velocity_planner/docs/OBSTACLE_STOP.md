# Obstacle Stop

Stops the vehicle when obstacles are detected inside a critical stop distance.

Inputs:
- `/input/path_with_lane_id` : `autoware_internal_planning_msgs/PathWithLaneId`
- `/perception/objects` : `autoware_perception_msgs/PredictedObjects`
- `/velocity/limit` : `autoware_internal_planning_msgs/VelocityLimit`

Output:
- `/output/motion_velocity_path` : `autoware_internal_planning_msgs/PathWithLaneId`

Parameters:
- `~stop_distance` : default 3.0 m, base clearance kept in front of a static obstacle
- `~reaction_time` : default 0.8 s, ego-speed reaction buffer
- `~comfortable_deceleration` : default 3.0 m/s², deceleration used for braking-distance clearance
- `~object_size_margin` : default 0.0 m, optional fixed object-size safety margin
- `~slow_dynamic_stop_speed_threshold_mps` : default 1.0 m/s, maximum object speed still treated as a distance-based hard stop target
- `~warning_distance` : default 6.0 m
- `~min_stop_velocity_mps` : default 0.5 m/s

Behavior:
- Evaluates the closest static or very slow object distance to each path point.
- Faster leading objects are left to dynamic stop/TTC limiting instead of creating a fixed stop line.
- The stop line is placed before the object using:
  `stop_distance + ego_speed * reaction_time + ego_speed^2 / (2 * comfortable_deceleration) + object_size_margin`.
- When a point reaches the computed stop line, the rest of the path is set to 0 m/s.
- When within `warning_distance`, the point speed is lowered to at most `min_stop_velocity_mps`.
- Global `VelocityLimit.max_velocity` is also enforced.
