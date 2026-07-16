This document lists the topics and messages used by `behavior_path_planner` that the `motion_velocity_planner` module should reference when porting from ROS2 to ROS1.

- Subscribed topics (examples):
  - `/input/path_with_lane_id` : `autoware_internal_planning_msgs/PathWithLaneId`
  - `/odometry` : `nav_msgs/Odometry`
  - `/perception/objects` : `autoware_perception_msgs/PredictedObjects`
  - `/velocity/limit` : `autoware_internal_planning_msgs/VelocityLimit`

- Published topics (examples):
  - `/output/motion_velocity_path` : `autoware_planning_msgs/PathWithLaneId`

Refer to `behavior_path_planner` for specific message fields and expected behaviors.
