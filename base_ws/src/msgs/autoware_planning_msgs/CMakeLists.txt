cmake_minimum_required(VERSION 3.0.2)
project(autoware_planning_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  std_msgs
  unique_identifier_msgs
)

add_message_files(
  FILES
  LaneletPrimitive.msg
  LaneletSegment.msg
  LaneletRoute.msg
  PathPoint.msg
  Path.msg
  PoseWithUuidStamped.msg
  TrajectoryPoint.msg
  Trajectory.msg
  RouteState.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  unique_identifier_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  geometry_msgs
  std_msgs
  unique_identifier_msgs
)
