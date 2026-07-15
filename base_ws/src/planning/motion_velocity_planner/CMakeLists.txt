cmake_minimum_required(VERSION 3.0.2)
project(motion_velocity_planner)

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
  scripts/motion_velocity_planner.py
  scripts/obstacle_velocity_limiter.py
  scripts/out_of_lane.py
  scripts/obstacle_cruise.py
  scripts/obstacle_stop.py
  scripts/obstacle_slow_down.py
  scripts/dynamic_obstacle_stop.py
  scripts/boundary_departure_prevention.py
  scripts/validation/run_motion_velocity_planner_cases.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
