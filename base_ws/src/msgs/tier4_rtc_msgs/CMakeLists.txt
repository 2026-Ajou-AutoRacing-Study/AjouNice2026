cmake_minimum_required(VERSION 3.0.2)
project(tier4_rtc_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  unique_identifier_msgs
)

add_message_files(
  FILES
  Command.msg
  Module.msg
  State.msg
  CooperateCommand.msg
  CooperateResponse.msg
  CooperateStatus.msg
  CooperateStatusArray.msg
)

generate_messages(
  DEPENDENCIES
  unique_identifier_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  unique_identifier_msgs
)
