cmake_minimum_required(VERSION 3.0.2)
project(localization_manager)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  morai_msgs
  nav_msgs
  rospy
  tf
  tf2_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/map_camera_init_broadcaster.py
  scripts/kinematic_state_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

