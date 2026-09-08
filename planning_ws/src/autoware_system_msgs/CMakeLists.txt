cmake_minimum_required(VERSION 3.0.2)
project(autoware_system_msgs)

find_package(catkin REQUIRED COMPONENTS
  autoware_common_msgs
  builtin_interfaces
  diagnostic_msgs
  geometry_msgs
  message_generation
  nav_msgs
  std_msgs
  unique_identifier_msgs
)

add_message_files(FILES
  AutowareState.msg
  HazardStatus.msg
  HazardStatusStamped.msg
)

add_service_files(FILES
  ChangeAutowareControl.srv
  ChangeOperationMode.srv
)

generate_messages(DEPENDENCIES
  autoware_common_msgs
  builtin_interfaces
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  unique_identifier_msgs
)

catkin_package(CATKIN_DEPENDS
  autoware_common_msgs
  builtin_interfaces
  diagnostic_msgs
  geometry_msgs
  message_runtime
  nav_msgs
  std_msgs
  unique_identifier_msgs
)
