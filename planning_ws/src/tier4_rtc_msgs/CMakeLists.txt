cmake_minimum_required(VERSION 3.0.2)
project(tier4_rtc_msgs)

find_package(catkin REQUIRED COMPONENTS
  builtin_interfaces
  message_generation
  unique_identifier_msgs
)

add_message_files(FILES
  AutoModeStatus.msg
  AutoModeStatusArray.msg
  Command.msg
  CooperateCommand.msg
  CooperateResponse.msg
  CooperateStatus.msg
  CooperateStatusArray.msg
  Module.msg
  State.msg
)

add_service_files(FILES
  AutoMode.srv
  AutoModeWithModule.srv
  CooperateCommands.srv
)

generate_messages(DEPENDENCIES builtin_interfaces unique_identifier_msgs)

catkin_package(CATKIN_DEPENDS builtin_interfaces message_runtime unique_identifier_msgs)
