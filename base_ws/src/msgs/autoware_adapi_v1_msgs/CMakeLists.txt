cmake_minimum_required(VERSION 3.0.2)
project(autoware_adapi_v1_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

add_message_files(
  FILES
  OperationModeState.msg
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)
