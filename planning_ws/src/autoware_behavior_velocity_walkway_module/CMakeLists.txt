cmake_minimum_required(VERSION 3.14)
project(autoware_behavior_velocity_walkway_module)
set(CMAKE_CXX_STANDARD 17)
find_package(catkin REQUIRED COMPONENTS autoware_behavior_velocity_crosswalk_module autoware_behavior_velocity_planner_common autoware_lanelet2_extension autoware_motion_utils autoware_objects_of_interest_marker_interface autoware_planning_msgs autoware_route_handler autoware_trajectory autoware_utils autoware_utils_geometry autoware_utils_rclcpp autoware_vehicle_info_utils geometry_msgs pluginlib roscpp visualization_msgs)
find_package(fmt REQUIRED)
catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS autoware_behavior_velocity_crosswalk_module autoware_behavior_velocity_planner_common autoware_lanelet2_extension autoware_motion_utils autoware_objects_of_interest_marker_interface autoware_planning_msgs autoware_route_handler autoware_trajectory autoware_utils autoware_utils_geometry autoware_utils_rclcpp autoware_vehicle_info_utils geometry_msgs pluginlib roscpp visualization_msgs)
add_library(${PROJECT_NAME} SHARED
  src/debug.cpp
  src/experimental/manager.cpp
  src/experimental/scene_walkway.cpp
  src/manager.cpp
  src/scene_walkway.cpp)
target_include_directories(${PROJECT_NAME} BEFORE PUBLIC include src ${catkin_INCLUDE_DIRS})
target_compile_options(${PROJECT_NAME} PRIVATE -fmax-errors=5 -include ${CMAKE_CURRENT_SOURCE_DIR}/include/autoware/behavior_velocity_walkway_module/ros1_support.hpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} fmt::fmt)
target_link_options(${PROJECT_NAME} PRIVATE -Wl,--no-undefined)
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_full_walkway test/test_full_walkway.cpp)
  if(TARGET test_full_walkway)
    target_include_directories(test_full_walkway BEFORE PRIVATE include src ${catkin_INCLUDE_DIRS})
    target_compile_options(test_full_walkway PRIVATE -include ${CMAKE_CURRENT_SOURCE_DIR}/include/autoware/behavior_velocity_walkway_module/ros1_support.hpp)
    target_link_libraries(test_full_walkway ${PROJECT_NAME} ${catkin_LIBRARIES} fmt::fmt)
  endif()
endif()
