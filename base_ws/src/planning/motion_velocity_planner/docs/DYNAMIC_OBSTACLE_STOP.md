# Dynamic Obstacle Stop

Stops the vehicle when dynamic obstacles are detected within a critical distance.

Inputs:
- `/input/path_with_lane_id` : `autoware_internal_planning_msgs/PathWithLaneId`
- `/perception/objects` : `autoware_perception_msgs/PredictedObjects`
- `/velocity/limit` : `autoware_internal_planning_msgs/VelocityLimit`

Output:
- `/output/motion_velocity_path` : `autoware_internal_planning_msgs/PathWithLaneId`

Parameters:
- `~dynamic_stop_distance` : default 5.0 m
- `~warning_distance` : default 10.0 m
- `~min_velocity_mps` : default 0.5 m/s

Behavior:
- Checks predicted objects and identifies the closest relevant dynamic object.
- If any dynamic object is inside `dynamic_stop_distance`, the remainder of the path is forced to 0 m/s.
- If inside `warning_distance`, speed is reduced to at most `min_velocity_mps`.
- Global `VelocityLimit.max_velocity` is also enforced.

Notes:
- This implementation uses `PredictedObjects` labels to preferentially treat dynamic objects. In a real system, additional checks for object velocity and classification would improve robustness.
