cmake_minimum_required(VERSION 3.0.2)
project(velocity_planner)

find_package(catkin REQUIRED COMPONENTS
  autoware_adapi_v1_msgs
  autoware_internal_planning_msgs
  autoware_map_msgs
  autoware_perception_msgs
  autoware_planning_msgs
  autoware_vehicle_msgs
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  tier4_planning_msgs
)

catkin_package(
  CATKIN_DEPENDS
  autoware_adapi_v1_msgs
  autoware_internal_planning_msgs
  autoware_map_msgs
  autoware_perception_msgs
  autoware_planning_msgs
  autoware_vehicle_msgs
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  tier4_planning_msgs
)

catkin_install_python(
  PROGRAMS
  scripts/velocity_planner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
