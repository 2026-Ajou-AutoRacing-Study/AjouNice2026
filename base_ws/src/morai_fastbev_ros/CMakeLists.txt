cmake_minimum_required(VERSION 3.0.2)
project(morai_fastbev_ros)

find_package(catkin REQUIRED COMPONENTS
  autoware_perception_msgs
  geometry_msgs
  jsk_recognition_msgs
  message_filters
  morai_pointpainting_ros
  perception_object_pipeline
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS
  autoware_perception_msgs
  geometry_msgs
  jsk_recognition_msgs
  message_filters
  morai_pointpainting_ros
  perception_object_pipeline
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/fastbev_online_node.py
    scripts/benchmark_fastbev_runtime.py
    scripts/validate_fastbev_preprocessing.py
    scripts/run_fastbev.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
