cmake_minimum_required(VERSION 3.14)
project(autoware_behavior_velocity_crosswalk_module)
set(CMAKE_CXX_STANDARD 17)
find_package(catkin REQUIRED COMPONENTS roscpp pluginlib autoware_behavior_velocity_planner_common autoware_behavior_velocity_rtc_interface autoware_grid_map_utils autoware_internal_debug_msgs autoware_internal_planning_msgs autoware_interpolation autoware_lanelet2_extension autoware_lanelet2_utils autoware_motion_utils autoware_perception_msgs autoware_planning_msgs autoware_route_handler autoware_signal_processing autoware_utils autoware_utils_rclcpp autoware_vehicle_info_utils geometry_msgs grid_map_core grid_map_ros nav_msgs pcl_conversions sensor_msgs visualization_msgs autoware_rtc_interface autoware_trajectory autoware_objects_of_interest_marker_interface)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS roscpp pluginlib autoware_behavior_velocity_planner_common autoware_behavior_velocity_rtc_interface autoware_grid_map_utils autoware_internal_debug_msgs autoware_internal_planning_msgs autoware_interpolation autoware_lanelet2_extension autoware_lanelet2_utils autoware_motion_utils autoware_perception_msgs autoware_planning_msgs autoware_route_handler autoware_signal_processing autoware_utils autoware_utils_rclcpp autoware_vehicle_info_utils geometry_msgs grid_map_core grid_map_ros nav_msgs pcl_conversions sensor_msgs visualization_msgs autoware_rtc_interface autoware_trajectory autoware_objects_of_interest_marker_interface)
add_library(${PROJECT_NAME} SHARED
  src/debug.cpp
  src/experimental/manager.cpp
  src/experimental/occluded_crosswalk.cpp
  src/experimental/scene_crosswalk.cpp
  src/manager.cpp
  src/occluded_crosswalk.cpp
  src/parked_vehicles_stop.cpp
  src/scene_crosswalk.cpp
  src/util.cpp)
target_include_directories(${PROJECT_NAME} BEFORE PUBLIC include src ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_compile_options(${PROJECT_NAME} PRIVATE -fmax-errors=10
  -include ${CMAKE_CURRENT_SOURCE_DIR}/include/autoware/behavior_velocity_crosswalk_module/ros1_support.hpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${autoware_rtc_interface_LIBRARIES}
  ${autoware_trajectory_LIBRARIES} ${autoware_objects_of_interest_marker_interface_LIBRARIES})
target_link_options(${PROJECT_NAME} PRIVATE -Wl,--no-undefined)
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS scripts/time_to_collision_plotter.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY reference_ros2_tests/test_config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test_config)
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_full_crosswalk_interpolation reference_ros2_tests/test_crosswalk.cpp)
  target_link_libraries(test_full_crosswalk_interpolation ${PROJECT_NAME} gtest_main)
  catkin_add_gtest(test_full_crosswalk_plugins test/test_full_crosswalk_plugins.cpp)
  target_link_libraries(test_full_crosswalk_plugins ${PROJECT_NAME})
  option(AW149_FULL_HOST_TESTS "Build integration test after complete host installation" OFF)
  if(AW149_FULL_HOST_TESTS)
    find_package(autoware_behavior_velocity_planner REQUIRED)
    find_package(autoware_test_utils REQUIRED)
    catkin_add_gtest(test_full_crosswalk_host test/test_full_crosswalk_host.cpp)
    target_include_directories(test_full_crosswalk_host PRIVATE ${autoware_behavior_velocity_planner_INCLUDE_DIRS} ${autoware_test_utils_INCLUDE_DIRS})
    target_link_libraries(test_full_crosswalk_host ${PROJECT_NAME} ${autoware_behavior_velocity_planner_LIBRARIES} ${autoware_test_utils_LIBRARIES})
  endif()
endif()
