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
  tier4_rtc_msgs
  unique_identifier_msgs
  visualization_msgs
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
  tier4_rtc_msgs
  unique_identifier_msgs
  visualization_msgs
)

catkin_install_python(
  PROGRAMS
  scripts/behavior_path_planner.py
  scripts/bpp_lane_change_tools.py
  scripts/bpp_dynamic_avoidance_tools.py
  scripts/bpp_goal_planner_tools.py
  scripts/bpp_map_tools.py
  scripts/bpp_path_tools.py
  scripts/bpp_route_tools.py
  scripts/bpp_side_shift_tools.py
  scripts/bpp_start_planner_tools.py
  scripts/bpp_static_avoidance_tools.py
  scripts/dynamic_obstacle_avoidance_bpp.py
  scripts/goal_planner_bpp.py
  scripts/lane_change_bpp.py
  scripts/lane_following_bpp.py
  scripts/rtc_interface_ros1.py
  scripts/side_shift_bpp.py
  scripts/start_planner_bpp.py
  scripts/static_obstacle_avoidance_bpp.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_install_python(
    PROGRAMS
    validation/scripts/bpp_minimal_input_publisher.py
    validation/scripts/bpp_path_comparison_visualizer.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(
    DIRECTORY validation/launch validation/rviz
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/validation
  )

  catkin_add_nosetests(test/test_dynamic_avoidance_engine.py)
  catkin_add_nosetests(test/test_goal_planner_engine.py)
  catkin_add_nosetests(test/test_route_lane_change.py)
  catkin_add_nosetests(test/test_side_shift_engine.py)
  catkin_add_nosetests(test/test_start_planner_engine.py)
  catkin_add_nosetests(test/test_static_avoidance_engine.py)
endif()
