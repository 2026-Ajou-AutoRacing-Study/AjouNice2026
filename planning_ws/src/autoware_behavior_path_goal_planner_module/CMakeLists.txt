cmake_minimum_required(VERSION 3.14)
project(autoware_behavior_path_goal_planner_module)
find_package(catkin REQUIRED COMPONENTS roscpp pluginlib visualization_msgs
  autoware_behavior_path_planner_common autoware_behavior_path_lane_change_module
  autoware_bezier_sampler autoware_freespace_planning_algorithms
  autoware_lanelet2_utils autoware_motion_utils autoware_rtc_interface
  autoware_trajectory autoware_utils autoware_utils_rclcpp autoware_vehicle_info_utils
  autoware_deprecated_boundary_departure_checker)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(TBB CONFIG REQUIRED)
set(CMAKE_CXX_STANDARD 17)
catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp pluginlib visualization_msgs autoware_behavior_path_planner_common
  autoware_behavior_path_lane_change_module autoware_bezier_sampler
  autoware_freespace_planning_algorithms autoware_lanelet2_utils autoware_motion_utils
  autoware_rtc_interface autoware_trajectory autoware_utils autoware_utils_rclcpp autoware_vehicle_info_utils
  autoware_deprecated_boundary_departure_checker)
add_library(${PROJECT_NAME} SHARED
  src/pull_over_planner/pull_over_planner_base.cpp
  src/pull_over_planner/freespace_pull_over.cpp
  src/pull_over_planner/geometric_pull_over.cpp
  src/pull_over_planner/shift_pull_over.cpp
  src/pull_over_planner/bezier_pull_over.cpp
  src/default_fixed_goal_planner.cpp src/goal_searcher.cpp src/util.cpp
  src/goal_planner_module.cpp src/manager.cpp src/thread_data.cpp
  src/decision_state.cpp src/lane_change.cpp)
target_include_directories(${PROJECT_NAME} BEFORE PUBLIC include ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS})
target_compile_options(${PROJECT_NAME} PRIVATE -fmax-errors=5
  -include ${CMAKE_CURRENT_SOURCE_DIR}/include/autoware/behavior_path_goal_planner_module/ros1_logging.hpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} fmt::fmt TBB::tbb)
target_link_options(${PROJECT_NAME} PRIVATE -Wl,--no-undefined)
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(DIRECTORY config test_data DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  find_package(autoware_test_utils REQUIRED)
  find_package(yaml-cpp REQUIRED)
  catkin_add_gtest(test_full_goal_planning_factor test/test_planning_factor.cpp)
  if(TARGET test_full_goal_planning_factor)
    target_include_directories(test_full_goal_planning_factor BEFORE PRIVATE test/include include
      ${catkin_INCLUDE_DIRS} ${autoware_test_utils_INCLUDE_DIRS})
    target_link_libraries(test_full_goal_planning_factor ${PROJECT_NAME} ${catkin_LIBRARIES}
      ${autoware_test_utils_LIBRARIES} yaml-cpp)
  endif()
  catkin_add_gtest(test_full_goal_util test/test_util.cpp)
  if(TARGET test_full_goal_util)
    target_include_directories(test_full_goal_util BEFORE PRIVATE include
      ${catkin_INCLUDE_DIRS} ${autoware_test_utils_INCLUDE_DIRS})
    target_link_libraries(test_full_goal_util ${PROJECT_NAME} ${catkin_LIBRARIES}
      ${autoware_test_utils_LIBRARIES} yaml-cpp)
  endif()
  catkin_add_gtest(test_goal_lane_change_context test/test_lane_change.cpp)
  if(TARGET test_goal_lane_change_context)
    target_include_directories(test_goal_lane_change_context BEFORE PRIVATE include ${catkin_INCLUDE_DIRS})
    target_link_libraries(test_goal_lane_change_context ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_full_goal_manager test/test_manager_lifecycle.cpp)
  if(TARGET test_full_goal_manager)
    target_include_directories(test_full_goal_manager BEFORE PRIVATE include ${catkin_INCLUDE_DIRS})
    target_compile_options(test_full_goal_manager PRIVATE
      -include ${CMAKE_CURRENT_SOURCE_DIR}/include/autoware/behavior_path_goal_planner_module/ros1_logging.hpp)
    target_link_libraries(test_full_goal_manager ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_ros1_owned_goal_timer test/test_ros1_owned_timer.cpp)
  if(TARGET test_ros1_owned_goal_timer)
    target_include_directories(test_ros1_owned_goal_timer BEFORE PRIVATE include ${catkin_INCLUDE_DIRS})
    target_link_libraries(test_ros1_owned_goal_timer ${catkin_LIBRARIES})
  endif()
endif()

# Retain bounded adapter APIs and all existing regression tests.
target_sources(${PROJECT_NAME} PRIVATE
  src/freespace_bpp_adapter.cpp
  src/freespace_pull_over_core.cpp
  src/goal_candidate_area_filter.cpp
  src/goal_candidate_boundary_core.cpp
  src/goal_candidate_boundary_sampler.cpp
  src/goal_candidate_generator_core.cpp
  src/goal_candidate_object_counter.cpp
  src/goal_candidate_object_filter.cpp
  src/goal_candidate_ranker.cpp
  src/goal_candidate_route_search.cpp
  src/goal_candidate_safety_core.cpp
  src/goal_candidate_search_core.cpp
  src/goal_candidate_update_core.cpp
  src/lane_change_context_core.cpp
  src/path_decision_state_core.cpp
  src/pull_over_lane_adapter.cpp
  src/pull_over_path_core.cpp)
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_freespace_pull_over_core test/test_freespace_pull_over_core.cpp)
  if(TARGET test_freespace_pull_over_core)
    target_include_directories(test_freespace_pull_over_core BEFORE PRIVATE include ${catkin_INCLUDE_DIRS})
    target_link_libraries(test_freespace_pull_over_core gtest_main ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_goal_candidate test/test_goal_candidate.cpp)
  if(TARGET test_goal_candidate)
    target_include_directories(test_goal_candidate BEFORE PRIVATE include ${catkin_INCLUDE_DIRS})
    target_link_libraries(test_goal_candidate gtest_main ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_fixed_goal_planner_base test/test_fixed_goal_planner_base.cpp)
  if(TARGET test_fixed_goal_planner_base)
    target_include_directories(test_fixed_goal_planner_base BEFORE PRIVATE include ${catkin_INCLUDE_DIRS})
    target_link_libraries(test_fixed_goal_planner_base gtest_main ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_default_fixed_goal_planner test/test_default_fixed_goal_planner.cpp)
  if(TARGET test_default_fixed_goal_planner)
    target_include_directories(test_default_fixed_goal_planner BEFORE PRIVATE include ${catkin_INCLUDE_DIRS})
    target_link_libraries(test_default_fixed_goal_planner gtest_main ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_goal_candidate_ranker test/test_goal_candidate_ranker.cpp)
  if(TARGET test_goal_candidate_ranker)
    target_include_directories(test_goal_candidate_ranker BEFORE PRIVATE include ${catkin_INCLUDE_DIRS})
    target_link_libraries(test_goal_candidate_ranker gtest_main ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_goal_planner_parameters test/test_goal_planner_parameters.cpp)
  if(TARGET test_goal_planner_parameters)
    target_include_directories(test_goal_planner_parameters BEFORE PRIVATE include ${catkin_INCLUDE_DIRS})
    target_link_libraries(test_goal_planner_parameters gtest_main ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_goal_candidate_safety_core test/test_goal_candidate_safety_core.cpp)
  if(TARGET test_goal_candidate_safety_core)
    target_include_directories(test_goal_candidate_safety_core BEFORE PRIVATE include ${catkin_INCLUDE_DIRS})
    target_link_libraries(test_goal_candidate_safety_core gtest_main ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_goal_candidate_object_filter test/test_goal_candidate_object_filter.cpp)
  if(TARGET test_goal_candidate_object_filter)
    target_include_directories(test_goal_candidate_object_filter BEFORE PRIVATE include ${catkin_INCLUDE_DIRS})
    target_link_libraries(test_goal_candidate_object_filter gtest_main ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_goal_candidate_boundary_core test/test_goal_candidate_boundary_core.cpp)
  if(TARGET test_goal_candidate_boundary_core)
    target_include_directories(test_goal_candidate_boundary_core BEFORE PRIVATE include ${catkin_INCLUDE_DIRS})
    target_link_libraries(test_goal_candidate_boundary_core gtest_main ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_goal_candidate_area_filter test/test_goal_candidate_area_filter.cpp)
  if(TARGET test_goal_candidate_area_filter)
    target_include_directories(test_goal_candidate_area_filter BEFORE PRIVATE include ${catkin_INCLUDE_DIRS})
    target_link_libraries(test_goal_candidate_area_filter gtest_main ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_goal_candidate_generator_core test/test_goal_candidate_generator_core.cpp)
  if(TARGET test_goal_candidate_generator_core)
    target_include_directories(test_goal_candidate_generator_core BEFORE PRIVATE include ${catkin_INCLUDE_DIRS})
    target_link_libraries(test_goal_candidate_generator_core gtest_main ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_goal_candidate_boundary_sampler test/test_goal_candidate_boundary_sampler.cpp)
  if(TARGET test_goal_candidate_boundary_sampler)
    target_include_directories(test_goal_candidate_boundary_sampler BEFORE PRIVATE include ${catkin_INCLUDE_DIRS})
    target_link_libraries(test_goal_candidate_boundary_sampler gtest_main ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_goal_candidate_search_core test/test_goal_candidate_search_core.cpp)
  if(TARGET test_goal_candidate_search_core)
    target_include_directories(test_goal_candidate_search_core BEFORE PRIVATE include ${catkin_INCLUDE_DIRS})
    target_link_libraries(test_goal_candidate_search_core gtest_main ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_goal_candidate_route_search test/test_goal_candidate_route_search.cpp)
  if(TARGET test_goal_candidate_route_search)
    target_include_directories(test_goal_candidate_route_search BEFORE PRIVATE include ${catkin_INCLUDE_DIRS})
    target_link_libraries(test_goal_candidate_route_search gtest_main ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_goal_candidate_object_counter test/test_goal_candidate_object_counter.cpp)
  if(TARGET test_goal_candidate_object_counter)
    target_include_directories(test_goal_candidate_object_counter BEFORE PRIVATE include ${catkin_INCLUDE_DIRS})
    target_link_libraries(test_goal_candidate_object_counter gtest_main ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_goal_candidate_update_core test/test_goal_candidate_update_core.cpp)
  if(TARGET test_goal_candidate_update_core)
    target_include_directories(test_goal_candidate_update_core BEFORE PRIVATE include ${catkin_INCLUDE_DIRS})
    target_link_libraries(test_goal_candidate_update_core gtest_main ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
endif()

find_package(autoware_utils_rclcpp REQUIRED)
include_directories(BEFORE ${autoware_utils_rclcpp_INCLUDE_DIRS})
