cmake_minimum_required(VERSION 3.0.2)
project(tier4_system_msgs)

find_package(catkin REQUIRED COMPONENTS
  autoware_common_msgs
  builtin_interfaces
  diagnostic_msgs
  geometry_msgs
  std_msgs
  message_generation
)

add_message_files(DIRECTORY msg FILES
  AutowareState.msg
  DiagGraphStruct.msg
  DiagGraphStatus.msg
  DiagNodeStruct.msg
  DiagNodeStatus.msg
  DiagLeafStruct.msg
  DiagLeafStatus.msg
  DiagLinkStruct.msg
  DiagLinkStatus.msg
  OperationModeAvailability.msg
  EmergencyGoalsClearCommand.msg
  EmergencyGoalsStamped.msg
  EmergencyHoldingState.msg
  EmergencyState.msg
  EmergencyStateStamped.msg
  HazardStatus.msg
  HazardStatusStamped.msg
  ServiceLog.msg
  ModeChangeAvailable.msg
  MrmBehaviorStatus.msg
  CommandSourceStatus.msg
  CommandModeAvailability.msg
  CommandModeAvailabilityItem.msg
  CommandModeRequest.msg
  CommandModeRequestItem.msg
  CommandModeStatus.msg
  CommandModeStatusItem.msg

)
add_service_files(DIRECTORY srv FILES
  OperateMrm.srv
  ChangeOperationMode.srv
  ChangeAutowareControl.srv
  ChangeTopicRelayControl.srv
  ResetDiagGraph.srv
  SelectCommandSource.srv

)
generate_messages(DEPENDENCIES
  autoware_common_msgs
  builtin_interfaces
  diagnostic_msgs
  geometry_msgs
  std_msgs
)
catkin_package(CATKIN_DEPENDS
  autoware_common_msgs
  builtin_interfaces
  diagnostic_msgs
  geometry_msgs
  std_msgs
  message_runtime
)
