cmake_minimum_required(VERSION 3.14)
project(autoware_behavior_velocity_blind_spot_module)
set(CMAKE_CXX_STANDARD 17)
find_package(catkin REQUIRED COMPONENTS autoware_behavior_velocity_planner_common autoware_behavior_velocity_rtc_interface autoware_internal_planning_msgs autoware_lanelet2_utils autoware_motion_utils autoware_object_recognition_utils autoware_objects_of_interest_marker_interface autoware_perception_msgs autoware_planning_msgs autoware_route_handler autoware_test_utils autoware_trajectory autoware_utils autoware_utils_rclcpp geometry_msgs pluginlib roscpp tf2 tier4_planning_msgs visualization_msgs autoware_rtc_interface autoware_utils_geometry std_msgs tf2_geometry_msgs)
find_package(fmt REQUIRED)
catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS autoware_behavior_velocity_planner_common autoware_behavior_velocity_rtc_interface autoware_internal_planning_msgs autoware_lanelet2_utils autoware_motion_utils autoware_object_recognition_utils autoware_objects_of_interest_marker_interface autoware_perception_msgs autoware_planning_msgs autoware_route_handler autoware_test_utils autoware_trajectory autoware_utils autoware_utils_rclcpp geometry_msgs pluginlib roscpp tf2 tier4_planning_msgs visualization_msgs autoware_rtc_interface autoware_utils_geometry std_msgs tf2_geometry_msgs)
add_library(${PROJECT_NAME} SHARED
  src/debug.cpp
  src/decisions.cpp
  src/experimental/decisions.cpp
  src/experimental/manager.cpp
  src/experimental/scene.cpp
  src/experimental/time_to_collision.cpp
  src/manager.cpp
  src/parameter.cpp
  src/scene.cpp
  src/time_to_collision.cpp
  src/util.cpp)
target_include_directories(${PROJECT_NAME} BEFORE PUBLIC include src ${catkin_INCLUDE_DIRS})
target_compile_options(${PROJECT_NAME} PRIVATE -fmax-errors=5 -include ${CMAKE_CURRENT_SOURCE_DIR}/include/autoware/behavior_velocity_blind_spot_module/ros1_support.hpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} fmt::fmt)
target_link_options(${PROJECT_NAME} PRIVATE -Wl,--no-undefined)
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(DIRECTORY config test_data DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  # Original test_util.cpp is also disabled upstream: it references removed
  # APIs/parameters. Retained in reference_ros2_tests, never counted as pass.
  catkin_add_gtest(test_current_blind_spot test/test_current_blind_spot.cpp)
  if(TARGET test_current_blind_spot)
    target_compile_definitions(test_current_blind_spot PRIVATE
      AW149_BVP_MODULE_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/../autoware_launch/config/ros1/planning/behavior_velocity_modules.yaml")
    target_link_libraries(test_current_blind_spot ${PROJECT_NAME})
  endif()
  option(AW149_FULL_HOST_TESTS "Build real-host tests after host installation" OFF)
  if(AW149_FULL_HOST_TESTS)
    find_package(autoware_behavior_velocity_planner REQUIRED)
    catkin_add_gtest(test_full_blind_spot_host test/test_full_blind_spot_host.cpp)
    target_include_directories(test_full_blind_spot_host PRIVATE ${autoware_behavior_velocity_planner_INCLUDE_DIRS})
    target_link_libraries(test_full_blind_spot_host ${PROJECT_NAME} ${autoware_behavior_velocity_planner_LIBRARIES} ${catkin_LIBRARIES})
  endif()
endif()
