cmake_minimum_required(VERSION 3.0.2)
project(autoware_motion_velocity_obstacle_cruise_module)

# Candidate-only ROS1 projection under R019. Every non-catkin Autoware
# provider is explicitly injected as an installed provider, never sourced from
# the ROS2 baseline.
find_package(catkin REQUIRED COMPONENTS
  autoware_internal_debug_msgs autoware_internal_planning_msgs
  autoware_perception_msgs autoware_planning_msgs geometry_msgs nav_msgs
  pluginlib roscpp sensor_msgs std_msgs tf2 tf2_eigen tf2_ros unique_identifier_msgs
  visualization_msgs)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include("${CMAKE_CURRENT_LIST_DIR}/../cmake/AW149Providers.cmake")
aw149_provider(INTERPOLATION autoware_interpolation autoware_interpolation)
aw149_provider(MARKER_INTERFACE autoware_objects_of_interest_marker_interface autoware_objects_of_interest_marker_interface)
aw149_provider(MOTION_UTILS autoware_motion_utils autoware_motion_utils)
aw149_provider(MVP_COMMON autoware_motion_velocity_planner_common autoware_motion_velocity_planner_common)
aw149_provider(OBJECT_RECOGNITION_UTILS autoware_object_recognition_utils autoware_object_recognition_utils)
aw149_provider(OSQP osqp osqp)
aw149_provider(OSQP_INTERFACE autoware_osqp_interface autoware_osqp_interface)
aw149_provider(PLANNING_FACTOR_INTERFACE autoware_planning_factor_interface autoware_planning_factor_interface)
aw149_provider(SIGNAL_PROCESSING autoware_signal_processing lowpass_filters)
aw149_provider(UNIVERSE_UTILS autoware_universe_utils NONE)
aw149_provider(UNIVERSE_UTILS_DEBUG autoware_utils_debug autoware_utils_debug)
aw149_provider(VEHICLE_INFO autoware_vehicle_info_utils vehicle_info_utils)
set(MVP_COMMON_INCLUDE_DIR "" CACHE PATH "installed MVP common include directory")
set(MVP_COMMON_LIBRARY "" CACHE FILEPATH "installed MVP common library")
set(MOTION_UTILS_INCLUDE_DIR "" CACHE PATH "installed motion-utils include directory")
set(MOTION_UTILS_LIBRARY "" CACHE FILEPATH "installed motion-utils library")
set(OBJECT_RECOGNITION_UTILS_INCLUDE_DIR "" CACHE PATH "installed object-recognition-utils include directory")
set(OBJECT_RECOGNITION_UTILS_LIBRARY "" CACHE FILEPATH "installed object-recognition-utils library")
set(VEHICLE_INFO_INCLUDE_DIR "" CACHE PATH "installed vehicle-info include directory")
set(MARKER_INTERFACE_INCLUDE_DIR "" CACHE PATH "installed objects-of-interest marker interface include directory")
set(MARKER_INTERFACE_LIBRARY "" CACHE FILEPATH "installed objects-of-interest marker interface library")
set(SIGNAL_PROCESSING_INCLUDE_DIR "" CACHE PATH "installed signal-processing include directory")
set(SIGNAL_PROCESSING_LIBRARY "" CACHE FILEPATH "installed signal-processing library")
set(PLANNING_FACTOR_INTERFACE_INCLUDE_DIR "" CACHE PATH "installed planning-factor interface include directory")
set(PLANNING_FACTOR_INTERFACE_LIBRARY "" CACHE FILEPATH "installed planning-factor interface library")
set(OSQP_INTERFACE_INCLUDE_DIR "" CACHE PATH "installed OSQP interface include directory")
set(OSQP_INTERFACE_LIBRARY "" CACHE FILEPATH "installed OSQP interface library")
set(OSQP_INCLUDE_DIR "" CACHE PATH "installed native OSQP include directory")
set(OSQP_LIBRARY "" CACHE FILEPATH "installed native OSQP library")
set(INTERPOLATION_INCLUDE_DIR "" CACHE PATH "installed interpolation include directory")
set(INTERPOLATION_LIBRARY "" CACHE FILEPATH "installed interpolation library")
set(UNIVERSE_UTILS_INCLUDE_DIR "" CACHE PATH "installed autoware-universe-utils include directory")
set(UNIVERSE_UTILS_DEBUG_LIBRARY "" CACHE FILEPATH "installed autoware-utils debug library")
foreach(path IN ITEMS MVP_COMMON_INCLUDE_DIR MVP_COMMON_LIBRARY MOTION_UTILS_INCLUDE_DIR MOTION_UTILS_LIBRARY OBJECT_RECOGNITION_UTILS_INCLUDE_DIR OBJECT_RECOGNITION_UTILS_LIBRARY VEHICLE_INFO_INCLUDE_DIR MARKER_INTERFACE_INCLUDE_DIR MARKER_INTERFACE_LIBRARY SIGNAL_PROCESSING_INCLUDE_DIR SIGNAL_PROCESSING_LIBRARY PLANNING_FACTOR_INTERFACE_INCLUDE_DIR PLANNING_FACTOR_INTERFACE_LIBRARY OSQP_INTERFACE_INCLUDE_DIR OSQP_INTERFACE_LIBRARY OSQP_INCLUDE_DIR OSQP_LIBRARY INTERPOLATION_INCLUDE_DIR INTERPOLATION_LIBRARY UNIVERSE_UTILS_INCLUDE_DIR UNIVERSE_UTILS_DEBUG_LIBRARY)
  if(NOT EXISTS "${${path}}")
    message(FATAL_ERROR "Set ${path} to its staged ROS1 provider")
  endif()
endforeach()

catkin_package(LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS autoware_internal_debug_msgs autoware_internal_planning_msgs autoware_perception_msgs autoware_planning_msgs geometry_msgs nav_msgs pluginlib roscpp sensor_msgs tf2 tf2_eigen tf2_ros unique_identifier_msgs visualization_msgs)
include_directories(src ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
  ${MVP_COMMON_INCLUDE_DIR} ${MOTION_UTILS_INCLUDE_DIR} ${OBJECT_RECOGNITION_UTILS_INCLUDE_DIR}
  ${VEHICLE_INFO_INCLUDE_DIR} ${MARKER_INTERFACE_INCLUDE_DIR} ${SIGNAL_PROCESSING_INCLUDE_DIR}
  ${PLANNING_FACTOR_INTERFACE_INCLUDE_DIR} ${OSQP_INTERFACE_INCLUDE_DIR} ${OSQP_INCLUDE_DIR} ${INTERPOLATION_INCLUDE_DIR}
  ${UNIVERSE_UTILS_INCLUDE_DIR})
add_library(${PROJECT_NAME} SHARED
  src/body_longitudinal_filter.cpp
  src/obstacle_cruise_module.cpp
  src/lateral_collision_check.cpp
  src/ros1_compat.cpp
  src/optimization_based_planner/optimization_based_planner.cpp
  src/optimization_based_planner/velocity_optimizer.cpp
  src/pid_based_planner/pid_based_planner.cpp)
target_link_libraries(${PROJECT_NAME}
  ${MVP_COMMON_LIBRARY} ${MOTION_UTILS_LIBRARY} ${OBJECT_RECOGNITION_UTILS_LIBRARY}
  ${MARKER_INTERFACE_LIBRARY} ${SIGNAL_PROCESSING_LIBRARY} ${PLANNING_FACTOR_INTERFACE_LIBRARY}
  ${OSQP_INTERFACE_LIBRARY} ${OSQP_LIBRARY} ${INTERPOLATION_LIBRARY} ${UNIVERSE_UTILS_DEBUG_LIBRARY}
  ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_lane_change_ownership test/test_lane_change_ownership.cpp)
  if(TARGET test_lane_change_ownership)
    target_link_libraries(test_lane_change_ownership ${catkin_LIBRARIES})
    target_compile_features(test_lane_change_ownership PUBLIC cxx_std_17)
  endif()
  add_rostest_gtest(test_pid_target_switch test/pid_target_switch.test test/test_pid_target_switch.cpp)
  if(TARGET test_pid_target_switch)
    target_link_libraries(test_pid_target_switch ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_lateral_collision_check test/test_lateral_collision_check.cpp)
  if(TARGET test_lateral_collision_check)
    target_link_libraries(test_lateral_collision_check ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_trajectory_index_mapping test/test_trajectory_index_mapping.cpp)
  if(TARGET test_trajectory_index_mapping)
    target_link_libraries(test_trajectory_index_mapping ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_body_longitudinal_filter test/test_body_longitudinal_filter.cpp)
  if(TARGET test_body_longitudinal_filter)
    target_link_libraries(test_body_longitudinal_filter ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_ros1_parameter_map test/test_ros1_parameter_map.cpp)
  if(TARGET test_ros1_parameter_map)
    target_link_libraries(test_ros1_parameter_map ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_pluginlib_load test/test_pluginlib_load.cpp)
  if(TARGET test_pluginlib_load)
    target_link_libraries(test_pluginlib_load ${PROJECT_NAME} ${catkin_LIBRARIES})
    target_compile_definitions(test_pluginlib_load PRIVATE OBSTACLE_CRUISE_LIBRARY_PATH="$<TARGET_FILE:${PROJECT_NAME}>")
  endif()
endif()
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(FILES package.xml plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
