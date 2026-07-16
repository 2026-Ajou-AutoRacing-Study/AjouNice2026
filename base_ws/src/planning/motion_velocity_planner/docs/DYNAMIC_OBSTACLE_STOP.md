# Dynamic Obstacle Stop

Stops the vehicle when dynamic obstacles are detected within a critical distance.

Inputs:
- `/input/path_with_lane_id` : `autoware_internal_planning_msgs/PathWithLaneId`
- `/perception/objects` : `autoware_perception_msgs/PredictedObjects`
- `/velocity/limit` : `autoware_internal_planning_msgs/VelocityLimit`

Output:
- `/output/motion_velocity_path` : `autoware_internal_planning_msgs/PathWithLaneId`

Parameters:
- `~dynamic_stop_distance` : default 5.0 m, base clearance kept in front of a dynamic obstacle
- `~reaction_time` : default 0.8 s, ego-speed reaction buffer
- `~comfortable_deceleration` : default 3.0 m/s², deceleration used for braking-distance clearance
- `~object_size_margin` : default 0.0 m, optional fixed object-size safety margin
- `~dynamic_object_time_margin` : default 1.0 s, extra clearance from positive ego/object relative speed
- `~slow_dynamic_stop_speed_threshold_mps` : default 1.0 m/s, dynamic objects at or below this speed are treated as hard stop targets
- `~dynamic_stop_ttc_threshold` : default 4.0 s, faster dynamic objects only trigger stop when time-to-collision is below this value
- `~dynamic_stop_min_relative_speed_mps` : default 1.0 m/s, minimum ego/object closing speed for TTC-based dynamic stop
- `~warning_distance` : default 10.0 m
- `~min_velocity_mps` : default 0.5 m/s

Behavior:
- Checks predicted objects and identifies the closest relevant dynamic object.
- Slow dynamic objects are handled as hard stop targets.
- Faster leading objects are not stopped for by default. They trigger dynamic stop only when ego is closing on them and TTC is short.
- The stop line is placed before the object using:
  `dynamic_stop_distance + ego_speed * reaction_time + ego_speed^2 / (2 * comfortable_deceleration) + object_size_margin + max(ego_speed - object_speed, 0) * dynamic_object_time_margin`.
- If any dynamic object reaches the computed stop line, the remainder of the path is forced to 0 m/s.
- If inside `warning_distance`, speed is reduced to at most `min_velocity_mps`.
- Global `VelocityLimit.max_velocity` is also enforced.

Notes:
- This keeps the behavior closer to Autoware planning simulator results: a fast leading vehicle is followed or velocity-limited unless TTC becomes critical, while a stopped or very slow object creates a stop line.
