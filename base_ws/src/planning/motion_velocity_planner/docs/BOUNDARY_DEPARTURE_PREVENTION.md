# Boundary Departure Prevention

Reduces speed when the vehicle is near lane boundaries or when path curvature suggests a boundary departure.

Inputs:
- `/input/path_with_lane_id` : `autoware_internal_planning_msgs/PathWithLaneId`
- `/input/vector_map` : `autoware_map_msgs/LaneletMapBin` (optional)
- `/perception/objects` : `autoware_perception_msgs/PredictedObjects` (optional)
- `/velocity/limit` : `autoware_internal_planning_msgs/VelocityLimit`

Output:
- `/output/motion_velocity_path` : `autoware_internal_planning_msgs/PathWithLaneId`

Parameters:
- `~boundary_distance` : default 1.0 m
- `~min_velocity_mps` : default 1.0 m/s
- `~curvature_warning` : default 0.15

Behavior:
- Evaluates path point curvature and slows the path if lateral velocity indicates a sharp turn.
- Applies a minimum speed floor of `min_velocity_mps`.
- Respects global `VelocityLimit.max_velocity` when present.

Notes:
- This module is a placeholder for boundary-based departure prevention. For a complete implementation, integrate lanelet boundary geometry and lane departure metrics.
