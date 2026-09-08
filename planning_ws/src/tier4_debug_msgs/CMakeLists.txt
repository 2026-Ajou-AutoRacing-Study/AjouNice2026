cmake_minimum_required(VERSION 3.0.2)
project(tier4_debug_msgs)

find_package(catkin REQUIRED COMPONENTS
  builtin_interfaces
  message_generation
)

add_message_files(FILES
  BoolStamped.msg
  Float32MultiArrayStamped.msg
  Float32Stamped.msg
  Float64MultiArrayStamped.msg
  Float64Stamped.msg
  Int32MultiArrayStamped.msg
  Int32Stamped.msg
  Int64MultiArrayStamped.msg
  Int64Stamped.msg
  MultiArrayDimension.msg
  MultiArrayLayout.msg
  ProcessingTimeNode.msg
  ProcessingTimeTree.msg
  StringStamped.msg
  SystemUsage.msg
  SystemUsageArray.msg
)

generate_messages(DEPENDENCIES
  builtin_interfaces
)

catkin_package(CATKIN_DEPENDS
  builtin_interfaces
  message_runtime
)
