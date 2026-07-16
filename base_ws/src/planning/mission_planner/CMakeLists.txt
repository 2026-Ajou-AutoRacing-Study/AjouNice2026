cmake_minimum_required(VERSION 3.0.2)
project(mission_planner)

find_package(catkin REQUIRED COMPONENTS
  autoware_adapi_v1_msgs
  autoware_common_msgs
  autoware_internal_debug_msgs
  autoware_internal_planning_msgs
  autoware_map_msgs
  autoware_planning_msgs
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS
  autoware_adapi_v1_msgs
  autoware_common_msgs
  autoware_internal_debug_msgs
  autoware_internal_planning_msgs
  autoware_map_msgs
  autoware_planning_msgs
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

catkin_install_python(
  PROGRAMS
  scripts/mission_planner.py
  scripts/mission_planner_minimal_input_publisher.py
  scripts/mission_planner_rviz_input.py
  scripts/mission_route_comparison_visualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES scripts/lanelet2_route_planner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch rviz docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
