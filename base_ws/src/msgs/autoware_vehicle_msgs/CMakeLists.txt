cmake_minimum_required(VERSION 3.0.2)
project(autoware_vehicle_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  FILES
  ActuationCommand.msg
  ActuationCommandStamped.msg
  ControlModeReport.msg
  Engage.msg
  GearCommand.msg
  GearReport.msg
  HazardLightsCommand.msg
  HazardLightsReport.msg
  SteeringReport.msg
  TurnIndicatorsCommand.msg
  TurnIndicatorsReport.msg
  VelocityReport.msg
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
