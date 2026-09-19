cmake_minimum_required(VERSION 3.0.2)
project(autoware_internal_msgs)

find_package(catkin REQUIRED COMPONENTS
  builtin_interfaces
  message_generation
  std_msgs
)

add_message_files(FILES
  PublishedTime.msg
  MissionRemainingDistanceTime.msg
  ResourceUsageReport.msg
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
