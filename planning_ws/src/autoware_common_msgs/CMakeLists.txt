cmake_minimum_required(VERSION 3.14)
project(autoware_common_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)


add_message_files(
  DIRECTORY msg
  FILES
  ResponseStatus.msg
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
)
