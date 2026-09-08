cmake_minimum_required(VERSION 3.0.2)
project(tier4_simulation_msgs)

find_package(catkin REQUIRED COMPONENTS
  autoware_perception_msgs
  builtin_interfaces
  geometry_msgs
  message_generation
  std_msgs
  unique_identifier_msgs
)

add_message_files(FILES
  FaultInjectionEvent.msg
  SimulationEvents.msg
  UserDefinedValue.msg
  UserDefinedValueType.msg
  DummyObject.msg
  DummyObjectInitialState.msg
)

generate_messages(DEPENDENCIES
  autoware_perception_msgs
  builtin_interfaces
  geometry_msgs
  std_msgs
  unique_identifier_msgs
)

catkin_package(CATKIN_DEPENDS
  autoware_perception_msgs
  builtin_interfaces
  geometry_msgs
  message_runtime
  std_msgs
  unique_identifier_msgs
)
