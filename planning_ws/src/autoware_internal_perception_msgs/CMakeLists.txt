cmake_minimum_required(VERSION 3.0.2)
project(autoware_internal_perception_msgs)

find_package(catkin REQUIRED COMPONENTS
  autoware_perception_msgs
  builtin_interfaces
  message_generation
  sensor_msgs
  std_msgs
)

add_message_files(FILES
  SegmentationMask.msg
)

generate_messages(DEPENDENCIES
  autoware_perception_msgs
  builtin_interfaces
  sensor_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  autoware_perception_msgs
  builtin_interfaces
  message_runtime
  sensor_msgs
  std_msgs
)
