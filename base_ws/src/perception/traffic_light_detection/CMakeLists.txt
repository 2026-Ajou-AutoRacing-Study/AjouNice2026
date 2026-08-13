cmake_minimum_required(VERSION 3.0.2)
project(traffic_light_detection)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  autoware_perception_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/traffic_light_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
