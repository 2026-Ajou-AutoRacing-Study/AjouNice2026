cmake_minimum_required(VERSION 3.0.2)
project(autoware_internal_planning_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  autoware_perception_msgs
  autoware_planning_msgs
  geometry_msgs
  std_msgs
  unique_identifier_msgs
)

add_message_files(
  FILES
  Scenario.msg
  PathPointWithLaneId.msg
  PathWithLaneId.msg
  VelocityLimitConstraints.msg
  VelocityLimit.msg
  RouteState.msg
  CandidateTrajectory.msg
  GeneratorInfo.msg
  CandidateTrajectories.msg
  ScoredCandidateTrajectory.msg
  ScoredCandidateTrajectories.msg
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
  autoware_planning_msgs
  geometry_msgs
  std_msgs
  unique_identifier_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  autoware_perception_msgs
  autoware_planning_msgs
  geometry_msgs
  std_msgs
  unique_identifier_msgs
)
