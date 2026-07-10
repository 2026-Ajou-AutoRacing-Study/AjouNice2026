cmake_minimum_required(VERSION 3.0.2)
project(autoware_internal_debug_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  FILES
  Float32MultiArrayStamped.msg
  Float32Stamped.msg
  Float64MultiArrayStamped.msg
  Float64Stamped.msg
  Int32Stamped.msg
  StringStamped.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
)
