cmake_minimum_required(VERSION 3.14)
project(autoware_behavior_velocity_occlusion_spot_module)
set(CMAKE_CXX_STANDARD 17)
find_package(catkin REQUIRED COMPONENTS autoware_behavior_velocity_planner_common autoware_behavior_velocity_rtc_interface autoware_grid_map_utils autoware_interpolation autoware_lanelet2_extension autoware_motion_utils autoware_objects_of_interest_marker_interface autoware_perception_msgs autoware_planning_msgs autoware_route_handler autoware_test_utils autoware_trajectory autoware_utils autoware_utils_rclcpp autoware_vehicle_info_utils geometry_msgs grid_map_ros nav_msgs pluginlib roscpp tf2 tf2_geometry_msgs visualization_msgs)
find_package(fmt REQUIRED)
find_package(OpenCV REQUIRED)
catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS autoware_behavior_velocity_planner_common autoware_behavior_velocity_rtc_interface autoware_grid_map_utils autoware_interpolation autoware_lanelet2_extension autoware_motion_utils autoware_objects_of_interest_marker_interface autoware_perception_msgs autoware_planning_msgs autoware_route_handler autoware_test_utils autoware_trajectory autoware_utils autoware_utils_rclcpp autoware_vehicle_info_utils geometry_msgs grid_map_ros nav_msgs pluginlib roscpp tf2 tf2_geometry_msgs visualization_msgs)
add_library(${PROJECT_NAME} SHARED
  src/debug.cpp
  src/experimental/manager.cpp
  src/experimental/scene_occlusion_spot.cpp
  src/experimental/utils.cpp
  src/grid_utils.cpp
  src/manager.cpp
  src/occlusion_spot_utils.cpp
  src/risk_predictive_braking.cpp
  src/scene_occlusion_spot.cpp)
target_include_directories(${PROJECT_NAME} BEFORE PUBLIC include src ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_compile_options(${PROJECT_NAME} PRIVATE -fmax-errors=5 -include ${CMAKE_CURRENT_SOURCE_DIR}/include/autoware/behavior_velocity_occlusion_spot_module/ros1_support.hpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} fmt::fmt)
target_link_options(${PROJECT_NAME} PRIVATE -Wl,--no-undefined)
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_full_occlusion_spot
    test/src/test_occlusion_spot_utils.cpp
    test/src/test_risk_predictive_braking.cpp
    test/src/test_grid_utils.cpp)
  target_include_directories(test_full_occlusion_spot PRIVATE test/src test/include)
  target_compile_options(test_full_occlusion_spot PRIVATE -include ${CMAKE_CURRENT_SOURCE_DIR}/include/autoware/behavior_velocity_occlusion_spot_module/ros1_support.hpp)
  target_link_libraries(test_full_occlusion_spot ${PROJECT_NAME} gtest_main)
  catkin_add_gtest(test_full_occlusion_plugins test/src/test_full_occlusion_plugins.cpp)
  target_link_libraries(test_full_occlusion_plugins ${PROJECT_NAME})
endif()
