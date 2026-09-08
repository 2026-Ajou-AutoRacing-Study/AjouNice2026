cmake_minimum_required(VERSION 3.0.2)
project(autoware_vehicle_msgs)

find_package(catkin REQUIRED COMPONENTS
  autoware_planning_msgs
  builtin_interfaces
  message_generation
  std_msgs
)

add_message_files(FILES
  ActuationCommand.msg
  ActuationCommandStamped.msg
  ActuationReport.msg
  ActuationReportStamped.msg
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

add_service_files(FILES
  ControlModeCommand.srv
)

generate_messages(DEPENDENCIES
  autoware_planning_msgs
  builtin_interfaces
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  autoware_planning_msgs
  builtin_interfaces
  message_runtime
  std_msgs
)
