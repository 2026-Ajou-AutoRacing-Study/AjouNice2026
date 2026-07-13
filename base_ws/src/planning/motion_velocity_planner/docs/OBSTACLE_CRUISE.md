# Obstacle Cruise

Adjusts cruising speed along a reference `PathWithLaneId` based on nearby obstacles.

Inputs:
- `/input/path_with_lane_id` : `autoware_internal_planning_msgs/PathWithLaneId`
- `/perception/objects` : `autoware_perception_msgs/PredictedObjects`
- `/velocity/limit` : `autoware_internal_planning_msgs/VelocityLimit` (optional)

Output:
- `/output/motion_velocity_path` : `autoware_internal_planning_msgs/PathWithLaneId` (modified velocities)

Parameters:
- `~cruise_speed_mps` : default 5.0
- `~safe_distance` : default 10.0 (m)
- `~slow_velocity_mps` : default 1.0 (m/s)

Behavior:
- For each path point, compute distance to nearest predicted object. If within `safe_distance`, reduce target velocity by linear interpolation down to `slow_velocity_mps`.
- Global `VelocityLimit.max_velocity` is applied if available.

Notes:
- This is a conservative, simple cruise limiter. For production, integrate predicted object velocities, object bounding boxes, and lane-based checks.
