cmake_minimum_required(VERSION 3.0.2)
project(viewer)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  rospy
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS nav_msgs rospy visualization_msgs
)

catkin_install_python(PROGRAMS
  scripts/current_vehicle_marker.py
  scripts/launch_history_plotjuggler.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
