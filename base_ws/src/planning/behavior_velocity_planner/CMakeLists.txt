cmake_minimum_required(VERSION 3.0.2)
project(behavior_velocity_planner)

find_package(catkin REQUIRED COMPONENTS
  autoware_internal_planning_msgs
  autoware_map_msgs
  autoware_perception_msgs
  autoware_planning_msgs
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tier4_planning_msgs
  tier4_v2x_msgs
  visualization_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  autoware_internal_planning_msgs
  autoware_map_msgs
  autoware_perception_msgs
  autoware_planning_msgs
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tier4_planning_msgs
  tier4_v2x_msgs
  visualization_msgs
)

catkin_install_python(
  PROGRAMS
  scripts/behavior_velocity_planner.py
  validation/scripts/replay_capture_ros1_bvp_blind_spot.py
  validation/scripts/replay_capture_ros1_bvp_restricted_area.py
  validation/scripts/replay_capture_ros1_bvp_advanced.py
  validation/scripts/bvp_blind_spot_comparison_visualizer.py
  validation/scripts/bvp_advanced_comparison_visualizer.py
  validation/scripts/bvp_intersection_comparison_visualizer.py
  validation/scripts/bvp_restricted_area_comparison_visualizer.py
  validation/scripts/bvp_speed_bump_comparison_visualizer.py
  validation/scripts/bvp_pedestrian_crossing_comparison_visualizer.py
  validation/scripts/bvp_stop_line_comparison_visualizer.py
  validation/scripts/bvp_traffic_light_comparison_visualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch docs validation/launch validation/rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_crosswalk_module.py)
  catkin_add_nosetests(test/test_detection_area_module.py)
  catkin_add_nosetests(test/test_blind_spot_module.py)
  catkin_add_nosetests(test/test_intersection_module.py)
  catkin_add_nosetests(test/test_merge_from_private_module.py)
  catkin_add_nosetests(test/test_no_drivable_lane_module.py)
  catkin_add_nosetests(test/test_no_stopping_area_module.py)
  catkin_add_nosetests(test/test_occlusion_spot_module.py)
  catkin_add_nosetests(test/test_run_out_module.py)
  catkin_add_nosetests(test/test_speed_bump_module.py)
  catkin_add_nosetests(test/test_stop_line_module.py)
  catkin_add_nosetests(test/test_traffic_light_module.py)
  catkin_add_nosetests(test/test_virtual_traffic_light_module.py)
  catkin_add_nosetests(test/test_walkway_module.py)
endif()
