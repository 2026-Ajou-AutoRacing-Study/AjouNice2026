cmake_minimum_required(VERSION 3.0.2)
project(autoware_adapi_v1_msgs)

find_package(catkin REQUIRED COMPONENTS
  builtin_interfaces
  geographic_msgs
  geometry_msgs
  shape_msgs
  std_msgs
  unique_identifier_msgs
  message_generation
)

add_message_files(DIRECTORY msg FILES
  AccelerationCommand.msg
  CooperationCommand.msg
  CooperationDecision.msg
  CooperationPolicy.msg
  CooperationStatus.msg
  DiagGraphStatus.msg
  DiagGraphStruct.msg
  DiagLeafStatus.msg
  DiagLeafStruct.msg
  DiagLinkStruct.msg
  DiagNodeStatus.msg
  DiagNodeStruct.msg
  DoorCommand.msg
  DoorLayout.msg
  DoorStatus.msg
  DoorStatusArray.msg
  DynamicObject.msg
  DynamicObjectArray.msg
  DynamicObjectKinematics.msg
  DynamicObjectPath.msg
  Gear.msg
  GearCommand.msg
  HazardLights.msg
  HazardLightsCommand.msg
  Heartbeat.msg
  KvString.msg
  LocalizationInitializationState.msg
  ManualControlMode.msg
  ManualControlModeStatus.msg
  ManualOperatorHeartbeat.msg
  MotionState.msg
  MrmDescription.msg
  MrmRequest.msg
  MrmRequestList.msg
  MrmState.msg
  ObjectClassification.msg
  OperationModeState.msg
  PedalsCommand.msg
  PlanningBehavior.msg
  PlanningSequence.msg
  ResponseStatus.msg
  Route.msg
  RouteData.msg
  RouteOption.msg
  RoutePrimitive.msg
  RouteSegment.msg
  RouteState.msg
  RtiState.msg
  SteeringCommand.msg
  SteeringFactor.msg
  SteeringFactorArray.msg
  TurnIndicators.msg
  TurnIndicatorsCommand.msg
  VehicleDimensions.msg
  VehicleKinematics.msg
  VehicleMetrics.msg
  VehicleSpecs.msg
  VehicleStatus.msg
  VelocityCommand.msg
  VelocityFactor.msg
  VelocityFactorArray.msg
)
add_service_files(DIRECTORY srv FILES
  AcceptStart.srv
  ChangeOperationMode.srv
  ClearRoute.srv
  GetCooperationPolicies.srv
  GetDoorLayout.srv
  GetVehicleDimensions.srv
  GetVehicleSpecs.srv
  InitializeLocalization.srv
  ListManualControlMode.srv
  ListMrmDescription.srv
  ResetDiagGraph.srv
  SelectManualControlMode.srv
  SendMrmRequest.srv
  SetCooperationCommands.srv
  SetCooperationPolicies.srv
  SetDoorCommand.srv
  SetRoute.srv
  SetRoutePoints.srv
)
generate_messages(DEPENDENCIES
  builtin_interfaces
  geographic_msgs
  geometry_msgs
  shape_msgs
  std_msgs
  unique_identifier_msgs
)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    builtin_interfaces
    geographic_msgs
    geometry_msgs
    shape_msgs
    std_msgs
    unique_identifier_msgs
    message_runtime
)
include_directories(include ${catkin_INCLUDE_DIRS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_bounded_sequence_validator
    test/test_bounded_sequence_validator.cpp)
  if(TARGET test_bounded_sequence_validator)
    add_dependencies(test_bounded_sequence_validator
      ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(test_bounded_sequence_validator ${catkin_LIBRARIES} gtest_main)
  endif()
endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
