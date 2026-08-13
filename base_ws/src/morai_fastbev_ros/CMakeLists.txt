cmake_minimum_required(VERSION 3.0.2)
project(morai_fastbev_ros)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  jsk_recognition_msgs
  message_filters
  morai_pointpainting_ros
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  jsk_recognition_msgs
  message_filters
  morai_pointpainting_ros
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/fastbev_online_node.py
    scripts/run_fastbev.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
