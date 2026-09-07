cmake_minimum_required(VERSION 3.0.2)
project(autoware_perception_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  std_msgs
  unique_identifier_msgs
)

add_message_files(
  FILES
  DetectedObjectKinematics.msg
  DetectedObject.msg
  DetectedObjects.msg
  ObjectClassification.msg
  Shape.msg
  PredictedPath.msg
  PredictedObjectKinematics.msg
  PredictedObject.msg
  PredictedObjects.msg
  TrafficLightElement.msg
  PredictedTrafficLightState.msg
  TrafficLightGroup.msg
  TrafficLightGroupArray.msg
  TrackedObjectKinematics.msg
  TrackedObject.msg
  TrackedObjects.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  unique_identifier_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  geometry_msgs
  std_msgs
  unique_identifier_msgs
)
