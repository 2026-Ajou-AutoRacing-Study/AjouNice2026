cmake_minimum_required(VERSION 3.14)
project(autoware_behavior_velocity_roundabout_module)
set(CMAKE_CXX_STANDARD 17)
find_package(catkin REQUIRED COMPONENTS autoware_behavior_velocity_intersection_module autoware_behavior_velocity_planner_common autoware_behavior_velocity_rtc_interface autoware_internal_debug_msgs autoware_internal_planning_msgs autoware_interpolation autoware_lanelet2_utils autoware_motion_utils autoware_object_recognition_utils autoware_objects_of_interest_marker_interface autoware_perception_msgs autoware_planning_msgs autoware_route_handler autoware_rtc_interface autoware_test_utils autoware_trajectory autoware_utils autoware_utils_rclcpp autoware_vehicle_info_utils geometry_msgs pluginlib roscpp sensor_msgs tf2 tf2_geometry_msgs tier4_planning_msgs visualization_msgs)
find_package(fmt REQUIRED)
catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS autoware_behavior_velocity_intersection_module autoware_behavior_velocity_planner_common autoware_behavior_velocity_rtc_interface autoware_internal_debug_msgs autoware_internal_planning_msgs autoware_interpolation autoware_lanelet2_utils autoware_motion_utils autoware_object_recognition_utils autoware_objects_of_interest_marker_interface autoware_perception_msgs autoware_planning_msgs autoware_route_handler autoware_rtc_interface autoware_test_utils autoware_trajectory autoware_utils autoware_utils_rclcpp autoware_vehicle_info_utils geometry_msgs pluginlib roscpp sensor_msgs tf2 tf2_geometry_msgs tier4_planning_msgs visualization_msgs)
add_library(${PROJECT_NAME} SHARED
  src/debug.cpp
  src/decision_result.cpp
  src/experimental/manager.cpp
  src/experimental/scene_roundabout.cpp
  src/experimental/scene_roundabout_collision.cpp
  src/experimental/scene_roundabout_prepare_data.cpp
  src/manager.cpp
  src/object_manager.cpp
  src/roundabout_lanelets.cpp
  src/scene_roundabout.cpp
  src/scene_roundabout_collision.cpp
  src/scene_roundabout_prepare_data.cpp)
target_include_directories(${PROJECT_NAME} BEFORE PUBLIC include src ${catkin_INCLUDE_DIRS})
target_compile_options(${PROJECT_NAME} PRIVATE -fmax-errors=5 -include ${CMAKE_CURRENT_SOURCE_DIR}/include/autoware/behavior_velocity_roundabout_module/ros1_support.hpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} fmt::fmt)
target_link_options(${PROJECT_NAME} PRIVATE -Wl,--no-undefined)
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS scripts/ttc.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_full_roundabout test/test_full_roundabout.cpp)
  if(TARGET test_full_roundabout)
    target_compile_definitions(test_full_roundabout PRIVATE
      AW149_BVP_MODULE_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/../autoware_launch/config/ros1/planning/behavior_velocity_modules.yaml")
    target_link_libraries(test_full_roundabout ${PROJECT_NAME})
  endif()
endif()
