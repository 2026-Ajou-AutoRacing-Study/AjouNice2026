cmake_minimum_required(VERSION 3.0.2)
project(trajectory_generator)

find_package(catkin REQUIRED COMPONENTS
  rospy geometry_msgs nav_msgs std_msgs visualization_msgs autoware_planning_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/trajectory_generator_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch artifacts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
