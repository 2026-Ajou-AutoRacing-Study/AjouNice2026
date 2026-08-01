cmake_minimum_required(VERSION 3.0.2)
project(tracking_prediction)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  morai_msgs
)

add_message_files(
  FILES
  TrackedObject.msg
  TrackedObjectArray.msg
  PredictedTrajectoryPoint.msg
  PredictedObject.msg
  PredictedObjectArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_runtime
  morai_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/tracking_node.py
  scripts/prediction_node.py
  scripts/pointpainting_tracking_node.py
  scripts/visualization_node.py
  scripts/gt_trajectory_markers_node.py
  scripts/gt_to_jsk_bbox_bridge.py
  scripts/ekf_tracking_node.py
  scripts/extrapolation_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
