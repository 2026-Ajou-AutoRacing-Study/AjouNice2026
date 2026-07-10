cmake_minimum_required(VERSION 3.0.2)
project(tier4_v2x_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

add_message_files(
  FILES
  KeyValue.msg
  InfrastructureCommand.msg
  InfrastructureCommandArray.msg
  VirtualTrafficLightState.msg
  VirtualTrafficLightStateArray.msg
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)
