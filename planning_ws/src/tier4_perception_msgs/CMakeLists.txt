cmake_minimum_required(VERSION 3.0.2)
project(tier4_perception_msgs)

find_package(catkin REQUIRED COMPONENTS
  autoware_perception_msgs
  builtin_interfaces
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
  unique_identifier_msgs
)

add_message_files(FILES
  DetectedObjectWithFeature.msg
  DetectedObjectsWithFeature.msg
  DynamicObject.msg
  DynamicObjectArray.msg
  Feature.msg
  PredictedPath.msg
  Semantic.msg
  Shape.msg
  State.msg
  TrafficLightElement.msg
  TrafficLightRoi.msg
  TrafficLightRoiArray.msg
  TrafficLight.msg
  TrafficLightArray.msg
)

generate_messages(DEPENDENCIES
  autoware_perception_msgs
  builtin_interfaces
  geometry_msgs
  sensor_msgs
  std_msgs
  unique_identifier_msgs
)

catkin_package(CATKIN_DEPENDS
  autoware_perception_msgs
  builtin_interfaces
  geometry_msgs
  message_runtime
  sensor_msgs
  std_msgs
  unique_identifier_msgs
)
