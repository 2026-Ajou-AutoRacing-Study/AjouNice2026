cmake_minimum_required(VERSION 3.0.2)
project(morai_pointpainting_ros)

find_package(catkin REQUIRED COMPONENTS
  autoware_perception_msgs
  cv_bridge
  geometry_msgs
  message_generation
  morai_msgs
  perception_object_pipeline
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

catkin_python_setup()

add_message_files(
  FILES
  PointPaintingDetection.msg
  PointPaintingDetectionArray.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  autoware_perception_msgs
  cv_bridge
  geometry_msgs
  message_runtime
  morai_msgs
  perception_object_pipeline
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/pointpainting_online_node.py
    scripts/check_pointpainting_output.py
    scripts/download_segmentation_weights.py
    scripts/run_pointpainting.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch config docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
