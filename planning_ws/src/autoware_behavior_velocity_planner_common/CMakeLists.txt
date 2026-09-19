cmake_minimum_required(VERSION 3.0.2)
project(autoware_behavior_velocity_planner_common)

find_package(catkin REQUIRED COMPONENTS
  autoware_internal_planning_msgs
  autoware_motion_utils
  autoware_perception_msgs
  autoware_planning_msgs
  autoware_vehicle_info_utils
  autoware_velocity_smoother
  geometry_msgs
  lanelet2_core
  nav_msgs
  roscpp
  tf2
  tf2_geometry_msgs
)
find_package(PCL REQUIRED)

include("${CMAKE_CURRENT_LIST_DIR}/../cmake/AW149Providers.cmake")
aw149_provider(OBJECTS_OF_INTEREST_MARKER autoware_objects_of_interest_marker_interface NONE)
aw149_provider(PLANNING_FACTOR_INTERFACE autoware_planning_factor_interface autoware_planning_factor_interface)
aw149_provider(AUTOWARE_UTILS_DEBUG autoware_utils_debug autoware_utils_debug)
aw149_provider(AUTOWARE_ROUTE_HANDLER autoware_route_handler autoware_route_handler)
aw149_provider(AUTOWARE_TRAJECTORY autoware_trajectory autoware_trajectory)
set(OBJECTS_OF_INTEREST_MARKER_INCLUDE_DIR "" CACHE PATH
  "Installed objects-of-interest marker-interface include directory for scene-module ABI")
if(NOT EXISTS "${OBJECTS_OF_INTEREST_MARKER_INCLUDE_DIR}/autoware/objects_of_interest_marker_interface/marker_data.hpp")
  message(FATAL_ERROR "Set OBJECTS_OF_INTEREST_MARKER_INCLUDE_DIR to the staged marker-interface install/include directory")
endif()
set(PLANNING_FACTOR_INTERFACE_INCLUDE_DIR "" CACHE PATH
  "ROS1-message-isolated planning-factor-interface include directory")
set(PLANNING_FACTOR_INTERFACE_LIBRARY "" CACHE FILEPATH
  "ROS1-message-isolated planning-factor-interface library")
set(AUTOWARE_UTILS_DEBUG_INCLUDE_DIR "" CACHE PATH
  "Staged autoware-utils-debug include directory")
set(AUTOWARE_UTILS_DEBUG_LIBRARY "" CACHE FILEPATH
  "Staged autoware-utils-debug library")
set(AUTOWARE_ROUTE_HANDLER_INCLUDE_DIR "" CACHE PATH
  "Installed ROS1 route-handler include directory for route/lane utilities")
set(AUTOWARE_ROUTE_HANDLER_LIBRARY "" CACHE FILEPATH
  "Installed ROS1 route-handler library for route/lane utilities")
set(AUTOWARE_TRAJECTORY_INCLUDE_DIR "" CACHE PATH
  "Installed ROS1 trajectory include directory for the experimental planner ABI")
set(AUTOWARE_TRAJECTORY_LIBRARY "" CACHE FILEPATH
  "Installed ROS1 trajectory library for the experimental planner ABI")
foreach(required_path IN ITEMS PLANNING_FACTOR_INTERFACE_INCLUDE_DIR PLANNING_FACTOR_INTERFACE_LIBRARY AUTOWARE_UTILS_DEBUG_INCLUDE_DIR AUTOWARE_UTILS_DEBUG_LIBRARY AUTOWARE_ROUTE_HANDLER_INCLUDE_DIR AUTOWARE_ROUTE_HANDLER_LIBRARY AUTOWARE_TRAJECTORY_INCLUDE_DIR AUTOWARE_TRAJECTORY_LIBRARY)
  if(NOT EXISTS "${${required_path}}")
    message(FATAL_ERROR "Set ${required_path} to the staged scene-manager provider artifact")
  endif()
endforeach()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES autoware_behavior_velocity_planner_common
  CATKIN_DEPENDS autoware_internal_planning_msgs autoware_motion_utils autoware_perception_msgs autoware_planning_msgs autoware_vehicle_info_utils autoware_velocity_smoother geometry_msgs lanelet2_core nav_msgs roscpp tf2 tf2_geometry_msgs
  DEPENDS PCL
)

include_directories(include ${OBJECTS_OF_INTEREST_MARKER_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_library(autoware_behavior_velocity_planner_common
  src/no_stopping_helpers.cpp
  src/detection_area.cpp
  src/utilization/boost_geometry_helper.cpp
  src/utilization/debug.cpp
  src/utilization/arc_lane_util.cpp
  src/utilization/path_utilization.cpp
  src/utilization/util.cpp
  src/utilization/trajectory_utils.cpp
  src/planner_data.cpp
  src/scene_module_interface.cpp
  src/experimental/scene_module_interface.cpp
)
target_link_libraries(autoware_behavior_velocity_planner_common ${catkin_LIBRARIES})
target_include_directories(autoware_behavior_velocity_planner_common PRIVATE
  ${AUTOWARE_UTILS_DEBUG_INCLUDE_DIR} ${AUTOWARE_ROUTE_HANDLER_INCLUDE_DIR} ${AUTOWARE_TRAJECTORY_INCLUDE_DIR})
target_link_libraries(autoware_behavior_velocity_planner_common "${AUTOWARE_ROUTE_HANDLER_LIBRARY}" "${AUTOWARE_TRAJECTORY_LIBRARY}")
target_compile_features(autoware_behavior_velocity_planner_common PUBLIC cxx_std_17)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_state_machine test/src/test_state_machine.cpp)
  if(TARGET test_state_machine)
    target_link_libraries(test_state_machine ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_arc_lane_util test/src/test_arc_lane_util.cpp)
  if(TARGET test_arc_lane_util)
    target_link_libraries(test_arc_lane_util autoware_behavior_velocity_planner_common ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_boost_geometry_helper test/test_boost_geometry_helper.cpp)
  if(TARGET test_boost_geometry_helper)
    target_link_libraries(test_boost_geometry_helper gtest_main autoware_behavior_velocity_planner_common ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_debug test/test_debug.cpp)
  if(TARGET test_debug)
    target_link_libraries(test_debug gtest_main autoware_behavior_velocity_planner_common ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_path_utilization test/src/test_path_utilization.cpp)
  if(TARGET test_path_utilization)
    target_link_libraries(test_path_utilization autoware_behavior_velocity_planner_common ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_util_math test/src/test_util.cpp)
  if(TARGET test_util_math)
    target_link_libraries(test_util_math autoware_behavior_velocity_planner_common ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_trajectory_utils test/src/test_trajectory_utils.cpp)
  if(TARGET test_trajectory_utils)
    target_link_libraries(test_trajectory_utils autoware_behavior_velocity_planner_common ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_plugin_wrapper test/test_plugin_wrapper.cpp)
  if(TARGET test_plugin_wrapper)
    target_link_libraries(test_plugin_wrapper gtest_main autoware_behavior_velocity_planner_common ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_planner_data test/test_planner_data.cpp)
  if(TARGET test_planner_data)
    target_link_libraries(test_planner_data autoware_behavior_velocity_planner_common ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_scene_module_interface test/test_scene_module_interface.cpp)
  if(TARGET test_scene_module_interface)
    target_link_libraries(test_scene_module_interface autoware_behavior_velocity_planner_common ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_scene_module_manager_interface test/test_scene_module_manager_interface.cpp)
  if(TARGET test_scene_module_manager_interface)
    # The manager is a header-only adapter boundary.  Keep its isolated
    # providers out of the common library's exported include/link interface;
    # a concrete manager must select the same ROS1-compatible providers itself.
    target_include_directories(test_scene_module_manager_interface PRIVATE
      ${PLANNING_FACTOR_INTERFACE_INCLUDE_DIR} ${AUTOWARE_UTILS_DEBUG_INCLUDE_DIR})
    target_link_libraries(test_scene_module_manager_interface
      autoware_behavior_velocity_planner_common
      "${PLANNING_FACTOR_INTERFACE_LIBRARY}" "${AUTOWARE_UTILS_DEBUG_LIBRARY}"
      ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_experimental_plugin_wrapper test/test_experimental_plugin_wrapper.cpp)
  if(TARGET test_experimental_plugin_wrapper)
    target_include_directories(test_experimental_plugin_wrapper PRIVATE
      ${PLANNING_FACTOR_INTERFACE_INCLUDE_DIR} ${AUTOWARE_UTILS_DEBUG_INCLUDE_DIR}
      ${AUTOWARE_TRAJECTORY_INCLUDE_DIR})
    target_link_libraries(test_experimental_plugin_wrapper
      autoware_behavior_velocity_planner_common
      "${PLANNING_FACTOR_INTERFACE_LIBRARY}" "${AUTOWARE_UTILS_DEBUG_LIBRARY}"
      "${AUTOWARE_TRAJECTORY_LIBRARY}" ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS autoware_behavior_velocity_planner_common
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
install(DIRECTORY config/ schema/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
