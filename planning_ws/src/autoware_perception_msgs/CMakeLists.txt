cmake_minimum_required(VERSION 3.0.2)
project(autoware_perception_msgs)

find_package(catkin REQUIRED COMPONENTS
  builtin_interfaces
  geometry_msgs
  message_generation
  std_msgs
  unique_identifier_msgs
)

add_message_files(FILES
  DetectedObject.msg
  DetectedObjectKinematics.msg
  DetectedObjects.msg
  ObjectClassification.msg
  PredictedObject.msg
  PredictedObjectKinematics.msg
  PredictedObjects.msg
  PredictedPath.msg
  PredictedTrafficLightState.msg
  Shape.msg
  TrackedObject.msg
  TrackedObjectKinematics.msg
  TrackedObjects.msg
  TrafficLightElement.msg
  TrafficLightGroup.msg
  TrafficLightGroupArray.msg
  TrafficSignalElement.msg
  TrafficSignal.msg
  TrafficSignalArray.msg
)

generate_messages(DEPENDENCIES
  builtin_interfaces
  geometry_msgs
  std_msgs
  unique_identifier_msgs
)

catkin_package(CATKIN_DEPENDS
  builtin_interfaces
  geometry_msgs
  message_runtime
  std_msgs
  unique_identifier_msgs
)
