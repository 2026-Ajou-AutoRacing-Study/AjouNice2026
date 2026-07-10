cmake_minimum_required(VERSION 3.0.2)
project(autoware_control_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

add_message_files(
  FILES
  Lateral.msg
  Longitudinal.msg
  Control.msg
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)
