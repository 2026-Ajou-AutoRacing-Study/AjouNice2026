cmake_minimum_required(VERSION 3.0.2)
project(autoware_planning_msgs)
find_package(catkin REQUIRED COMPONENTS
  autoware_common_msgs builtin_interfaces geometry_msgs message_generation nav_msgs std_msgs unique_identifier_msgs)
add_message_files(DIRECTORY msg FILES
  LaneletPrimitive.msg LaneletRoute.msg LaneletSegment.msg PoseWithUuidStamped.msg Trajectory.msg TrajectoryPoint.msg Path.msg PathPoint.msg RouteState.msg)
add_service_files(DIRECTORY srv FILES
  ClearRoute.srv SetLaneletRoute.srv SetPreferredPrimitive.srv SetWaypointRoute.srv)
generate_messages(DEPENDENCIES autoware_common_msgs builtin_interfaces geometry_msgs nav_msgs std_msgs unique_identifier_msgs)
catkin_package(CATKIN_DEPENDS autoware_common_msgs builtin_interfaces geometry_msgs message_runtime nav_msgs std_msgs unique_identifier_msgs)
