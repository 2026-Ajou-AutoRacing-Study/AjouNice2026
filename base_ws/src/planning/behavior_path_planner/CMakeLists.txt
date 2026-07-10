cmake_minimum_required(VERSION 3.0.2)
project(behavior_path_planner)

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
  scripts/behavior_path_planner.py
  scripts/bpp_minimal_input_publisher.py
  scripts/bpp_map_tools.py
  scripts/bpp_path_tools.py
  scripts/dynamic_obstacle_avoidance_bpp.py
  scripts/goal_planner_bpp.py
  scripts/lane_change_bpp.py
  scripts/lane_following_bpp.py
  scripts/side_shift_bpp.py
  scripts/start_planner_bpp.py
  scripts/static_obstacle_avoidance_bpp.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
