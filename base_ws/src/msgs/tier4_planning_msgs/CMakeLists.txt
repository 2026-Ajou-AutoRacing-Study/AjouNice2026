cmake_minimum_required(VERSION 3.0.2)
project(tier4_planning_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  autoware_perception_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  unique_identifier_msgs
)

add_message_files(
  FILES
  RerouteAvailability.msg
  LateralOffset.msg
  AvoidanceDebugMsg.msg
  AvoidanceDebugMsgArray.msg
  PathPoint.msg
  Path.msg
  TrajectoryPoint.msg
  Trajectory.msg
  StopFactor.msg
  StopReason.msg
  StopReasonArray.msg
  RouteState.msg
  ControlPoint.msg
  SafetyFactor.msg
  SafetyFactorArray.msg
  PlanningFactor.msg
  PlanningFactorArray.msg
  VelocityLimitClearCommand.msg
)

generate_messages(
  DEPENDENCIES
  autoware_perception_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  unique_identifier_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  autoware_perception_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  unique_identifier_msgs
)
