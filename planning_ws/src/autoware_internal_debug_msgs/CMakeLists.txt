cmake_minimum_required(VERSION 3.0.2)
project(autoware_internal_debug_msgs)

find_package(catkin REQUIRED COMPONENTS
  builtin_interfaces
  message_generation
  std_msgs
)

add_message_files(FILES
  BoolStamped.msg
  Float32Stamped.msg
  Float32MultiArrayStamped.msg
  Float64Stamped.msg
  Float64MultiArrayStamped.msg
  Int32Stamped.msg
  Int32MultiArrayStamped.msg
  Int64Stamped.msg
  Int64MultiArrayStamped.msg
  StringStamped.msg
  ProcessingTimeNode.msg
  ProcessingTimeTree.msg
  ServiceLog.msg
)

add_service_files(FILES
  String.srv
)

generate_messages(DEPENDENCIES
  builtin_interfaces
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  builtin_interfaces
  message_runtime
  std_msgs
)
