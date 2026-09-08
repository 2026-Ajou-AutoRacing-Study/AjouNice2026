cmake_minimum_required(VERSION 3.0.2)
project(tier4_control_msgs)

find_package(catkin REQUIRED COMPONENTS
  autoware_common_msgs
  builtin_interfaces
  message_generation
  std_msgs
)

add_message_files(FILES
  ExternalCommandSelectorMode.msg
  GateMode.msg
  IsPaused.msg
  IsStartRequested.msg
  IsStopped.msg
)

add_service_files(FILES
  ExternalCommandSelect.srv
  SetPause.srv
  SetStop.srv
)

generate_messages(DEPENDENCIES
  autoware_common_msgs
  builtin_interfaces
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  autoware_common_msgs
  builtin_interfaces
  message_runtime
  std_msgs
)
