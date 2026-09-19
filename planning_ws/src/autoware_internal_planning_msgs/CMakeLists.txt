cmake_minimum_required(VERSION 3.5)
project(autoware_internal_planning_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(catkin REQUIRED COMPONENTS
  autoware_common_msgs
  autoware_perception_msgs
  autoware_planning_msgs
  builtin_interfaces
  geometry_msgs
  std_msgs
  unique_identifier_msgs
  message_generation
)

add_message_files(DIRECTORY msg FILES
  CandidateTrajectories.msg
  CandidateTrajectory.msg
  GeneratorInfo.msg
  ScoredCandidateTrajectories.msg
  ScoredCandidateTrajectory.msg
  PathPointWithLaneId.msg
  PathWithLaneId.msg
  Scenario.msg
  ControlPoint.msg
  SafetyFactor.msg
  SafetyFactorArray.msg
  PlanningFactor.msg
  PlanningFactorArray.msg
  RouteState.msg
  VelocityLimit.msg
  VelocityLimitConstraints.msg
  VelocityLimitClearCommand.msg
)

add_service_files(DIRECTORY srv FILES
  ClearRoute.srv
  LoadPlugin.srv
  SetLaneletRoute.srv
  SetWaypointRoute.srv
  UnloadPlugin.srv
)

generate_messages(DEPENDENCIES
  autoware_common_msgs
  autoware_perception_msgs
  autoware_planning_msgs
  builtin_interfaces
  geometry_msgs
  std_msgs
  unique_identifier_msgs
)

catkin_package(CATKIN_DEPENDS
  autoware_common_msgs
  autoware_perception_msgs
  autoware_planning_msgs
  builtin_interfaces
  geometry_msgs
  std_msgs
  unique_identifier_msgs
  message_runtime
)
