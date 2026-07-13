# Obstacle Stop

Stops the vehicle when obstacles are detected inside a critical stop distance.

Inputs:
- `/input/path_with_lane_id` : `autoware_internal_planning_msgs/PathWithLaneId`
- `/perception/objects` : `autoware_perception_msgs/PredictedObjects`
- `/velocity/limit` : `autoware_internal_planning_msgs/VelocityLimit`

Output:
- `/output/motion_velocity_path` : `autoware_internal_planning_msgs/PathWithLaneId`

Parameters:
- `~stop_distance` : default 3.0 m
- `~warning_distance` : default 6.0 m
- `~min_stop_velocity_mps` : default 0.5 m/s

Behavior:
- Evaluates the closest object distance to each path point.
- When a point is within `stop_distance`, the rest of the path is set to 0 m/s.
- When within `warning_distance`, the point speed is lowered to at most `min_stop_velocity_mps`.
- Global `VelocityLimit.max_velocity` is also enforced.
