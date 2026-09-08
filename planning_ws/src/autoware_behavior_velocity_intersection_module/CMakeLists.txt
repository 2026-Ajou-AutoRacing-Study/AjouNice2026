cmake_minimum_required(VERSION 3.14)
project(autoware_behavior_velocity_intersection_module)
set(CMAKE_CXX_STANDARD 17)
find_package(catkin REQUIRED COMPONENTS autoware_behavior_velocity_planner_common autoware_behavior_velocity_rtc_interface autoware_internal_debug_msgs autoware_internal_planning_msgs autoware_interpolation autoware_lanelet2_utils autoware_motion_utils autoware_object_recognition_utils autoware_perception_msgs autoware_planning_msgs autoware_route_handler autoware_rtc_interface autoware_test_utils autoware_vehicle_info_utils geometry_msgs nav_msgs pluginlib tf2_geometry_msgs tier4_planning_msgs visualization_msgs roscpp autoware_utils autoware_utils_rclcpp autoware_trajectory autoware_objects_of_interest_marker_interface)
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS autoware_behavior_velocity_planner_common autoware_behavior_velocity_rtc_interface autoware_internal_debug_msgs autoware_internal_planning_msgs autoware_interpolation autoware_lanelet2_utils autoware_motion_utils autoware_object_recognition_utils autoware_perception_msgs autoware_planning_msgs autoware_route_handler autoware_rtc_interface autoware_test_utils autoware_vehicle_info_utils geometry_msgs nav_msgs pluginlib tf2_geometry_msgs tier4_planning_msgs visualization_msgs roscpp autoware_utils autoware_utils_rclcpp autoware_trajectory autoware_objects_of_interest_marker_interface)
add_library(${PROJECT_NAME} SHARED
  src/debug.cpp
  src/decision_result.cpp
  src/experimental/manager.cpp
  src/experimental/scene_intersection.cpp
  src/experimental/scene_intersection_collision.cpp
  src/experimental/scene_intersection_occlusion.cpp
  src/experimental/scene_intersection_prepare_data.cpp
  src/experimental/scene_intersection_stuck.cpp
  src/experimental/scene_merge_from_private_road.cpp
  src/intersection_lanelets.cpp
  src/manager.cpp
  src/object_manager.cpp
  src/scene_intersection.cpp
  src/scene_intersection_collision.cpp
  src/scene_intersection_occlusion.cpp
  src/scene_intersection_prepare_data.cpp
  src/scene_intersection_stuck.cpp
  src/scene_merge_from_private_road.cpp
  src/util.cpp)
target_include_directories(${PROJECT_NAME} BEFORE PUBLIC include src ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_compile_options(${PROJECT_NAME} PRIVATE -fmax-errors=6 -include ${CMAKE_CURRENT_SOURCE_DIR}/include/autoware/behavior_velocity_intersection_module/ros1_support.hpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} fmt::fmt)
target_link_options(${PROJECT_NAME} PRIVATE -Wl,--no-undefined)
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(DIRECTORY config test_data DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS scripts/ttc.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_full_intersection_util reference_ros2_tests/test_util.cpp)
  target_link_libraries(test_full_intersection_util ${PROJECT_NAME} gtest_main)
  catkin_add_gtest(test_full_intersection_plugins test/test_full_intersection_plugins.cpp)
  target_link_libraries(test_full_intersection_plugins ${PROJECT_NAME})
  option(AW149_FULL_HOST_TESTS "Build integration test after full host installation" OFF)
  if(AW149_FULL_HOST_TESTS)
    find_package(autoware_behavior_velocity_planner REQUIRED)
    catkin_add_gtest(test_full_intersection_host test/test_full_intersection_host.cpp)
    target_include_directories(test_full_intersection_host PRIVATE ${autoware_behavior_velocity_planner_INCLUDE_DIRS})
    target_link_libraries(test_full_intersection_host ${PROJECT_NAME} ${autoware_behavior_velocity_planner_LIBRARIES} ${catkin_LIBRARIES})
  endif()
endif()
