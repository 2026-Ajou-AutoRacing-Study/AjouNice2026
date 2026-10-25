cmake_minimum_required(VERSION 3.14)
project(autoware_behavior_velocity_traffic_light_module)
set(CMAKE_CXX_STANDARD 17)
find_package(catkin REQUIRED COMPONENTS autoware_adapi_v1_msgs autoware_behavior_velocity_planner_common autoware_behavior_velocity_rtc_interface autoware_internal_planning_msgs autoware_lanelet2_extension autoware_lanelet2_utils autoware_motion_utils autoware_objects_of_interest_marker_interface autoware_perception_msgs autoware_planning_msgs autoware_route_handler autoware_rtc_interface autoware_test_utils autoware_traffic_light_utils autoware_trajectory autoware_utils autoware_utils_geometry autoware_utils_rclcpp autoware_vehicle_info_utils geometry_msgs pluginlib roscpp tf2 tf2_eigen tf2_geometry_msgs tier4_planning_msgs visualization_msgs)
find_package(fmt REQUIRED)
catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS autoware_adapi_v1_msgs autoware_behavior_velocity_planner_common autoware_behavior_velocity_rtc_interface autoware_internal_planning_msgs autoware_lanelet2_extension autoware_lanelet2_utils autoware_motion_utils autoware_objects_of_interest_marker_interface autoware_perception_msgs autoware_planning_msgs autoware_route_handler autoware_rtc_interface autoware_test_utils autoware_traffic_light_utils autoware_trajectory autoware_utils autoware_utils_geometry autoware_utils_rclcpp autoware_vehicle_info_utils geometry_msgs pluginlib roscpp tf2 tf2_eigen tf2_geometry_msgs tier4_planning_msgs visualization_msgs)
add_library(${PROJECT_NAME} SHARED
  src/debug.cpp
  src/experimental/manager.cpp
  src/experimental/scene.cpp
  src/manager.cpp
  src/scene.cpp
  src/utils.cpp)
target_include_directories(${PROJECT_NAME} BEFORE PUBLIC include src ${catkin_INCLUDE_DIRS})
target_compile_options(${PROJECT_NAME} PRIVATE -fmax-errors=5 -include ${CMAKE_CURRENT_SOURCE_DIR}/include/autoware/behavior_velocity_traffic_light_module/ros1_support.hpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} fmt::fmt)
target_link_options(${PROJECT_NAME} PRIVATE -Wl,--no-undefined)
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS scripts/dilemma_zone_plotter.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  foreach(test test_original_yellow test_original_utils test_full_traffic_plugins test_signal_compliance_guard test_signal_compliance_module test_signal_yellow_switch_experimental)
    catkin_add_gtest(${test} test/${test}.cpp)
    if(TARGET ${test})
      target_include_directories(${test} BEFORE PRIVATE include src ${catkin_INCLUDE_DIRS})
      target_compile_options(${test} PRIVATE -include ${CMAKE_CURRENT_SOURCE_DIR}/include/autoware/behavior_velocity_traffic_light_module/ros1_support.hpp)
      target_link_libraries(${test} ${PROJECT_NAME} ${catkin_LIBRARIES} fmt::fmt gtest_main)
    endif()
  endforeach()
endif()
