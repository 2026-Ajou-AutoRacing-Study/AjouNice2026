# Obstacle Slow Down

Gradually reduces speed when obstacles are nearby without forcing an immediate stop.

Inputs:
- `/input/path_with_lane_id` : `autoware_internal_planning_msgs/PathWithLaneId`
- `/perception/objects` : `autoware_perception_msgs/PredictedObjects`
- `/velocity/limit` : `autoware_internal_planning_msgs/VelocityLimit`

Output:
- `/output/motion_velocity_path` : `autoware_internal_planning_msgs/PathWithLaneId`

Parameters:
- `~slowdown_distance` : default 15.0 m
- `~stop_distance` : default 3.0 m
- `~min_velocity_mps` : default 1.0 m/s
- `~cruise_velocity_mps` : default 5.0 m/s

Behavior:
- If the nearest obstacle is beyond `slowdown_distance`, the path retains cruise speed.
- If within `slowdown_distance`, the module interpolates speed down toward `min_velocity_mps`.
- If within `stop_distance`, the slowdown factor becomes 0 and path points may reach minimum speed.
- Global `VelocityLimit.max_velocity` is applied when available.
