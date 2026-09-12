cmake_minimum_required(VERSION 3.0.2)
project(autoware_behavior_path_static_obstacle_avoidance_module)

option(AW149_BPP_HOST_ABI "Compile Static Obstacle against the executable BPP Common ABI" ON)

find_package(catkin REQUIRED COMPONENTS geometry_msgs pluginlib roscpp sensor_msgs visualization_msgs)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_routing REQUIRED)

# Every non-system dependency is supplied explicitly by the candidate closure;
# discovery through an ambient AjouNice/ROS2 overlay is prohibited.
include("${CMAKE_CURRENT_LIST_DIR}/../cmake/AW149Providers.cmake")
aw149_provider(ADAPI_V1_MSGS autoware_adapi_v1_msgs NONE)
aw149_provider(AUTOWARE_UTILS_DEBUG autoware_utils_debug autoware_utils_debug)
aw149_provider(BOUNDARY_DEPARTURE_CHECKER autoware_deprecated_boundary_departure_checker boundary_departure_checker)
aw149_provider(BOUNDARY_DEPARTURE_CHECKER_UTILS autoware_deprecated_boundary_departure_checker boundary_departure_checker_utils)
aw149_provider(BPP_COMMON autoware_behavior_path_planner_common autoware_behavior_path_planner_common)
aw149_provider(FREESPACE_ALGORITHMS autoware_freespace_planning_algorithms autoware_freespace_planning_algorithms)
aw149_provider(LANELET2_EXTENSION autoware_lanelet2_extension autoware_lanelet2_extension_lib)
aw149_provider(LANELET2_UTILS autoware_lanelet2_utils autoware_lanelet2_utils)
aw149_provider(MAGIC_ENUM autoware_behavior_path_lane_change_module NONE)
aw149_provider(MOTION_UTILS autoware_motion_utils autoware_motion_utils)
aw149_provider(OBJECTS_OF_INTEREST autoware_objects_of_interest_marker_interface autoware_objects_of_interest_marker_interface)
aw149_provider(OBJECT_RECOGNITION_UTILS autoware_object_recognition_utils autoware_object_recognition_utils)
aw149_provider(PLANNING_FACTOR_INTERFACE autoware_planning_factor_interface autoware_planning_factor_interface)
aw149_provider(ROUTE_HANDLER autoware_route_handler autoware_route_handler)
aw149_provider(RTC_INTERFACE autoware_rtc_interface autoware_rtc_interface)
aw149_provider(TIER4_PLANNING_MSGS tier4_planning_msgs NONE)
aw149_provider(TRAFFIC_LIGHT_UTILS autoware_traffic_light_utils autoware_traffic_light_utils)
aw149_provider(UNIVERSE_UTILS autoware_universe_utils NONE)
aw149_provider(VEHICLE_INFO_UTILS autoware_vehicle_info_utils vehicle_info_utils)
foreach(provider IN ITEMS
  BPP_COMMON_INCLUDE_DIR BPP_COMMON_LIBRARY
  BOUNDARY_DEPARTURE_CHECKER_INCLUDE_DIR
  BOUNDARY_DEPARTURE_CHECKER_LIBRARY BOUNDARY_DEPARTURE_CHECKER_UTILS_LIBRARY
  AUTOWARE_UTILS_DEBUG_LIBRARY
  OBJECT_RECOGNITION_UTILS_INCLUDE_DIR OBJECT_RECOGNITION_UTILS_LIBRARY TRAFFIC_LIGHT_UTILS_LIBRARY
  MAGIC_ENUM_INCLUDE_DIR
  ADAPI_V1_MSGS_INCLUDE_DIR
  TIER4_PLANNING_MSGS_INCLUDE_DIR
  FREESPACE_ALGORITHMS_INCLUDE_DIR
  LANELET2_UTILS_INCLUDE_DIR LANELET2_UTILS_LIBRARY
  LANELET2_EXTENSION_INCLUDE_DIR LANELET2_EXTENSION_LIBRARY
  MOTION_UTILS_INCLUDE_DIR MOTION_UTILS_LIBRARY
  ROUTE_HANDLER_INCLUDE_DIR ROUTE_HANDLER_LIBRARY
  RTC_INTERFACE_INCLUDE_DIR RTC_INTERFACE_LIBRARY
  PLANNING_FACTOR_INTERFACE_INCLUDE_DIR PLANNING_FACTOR_INTERFACE_LIBRARY
  OBJECTS_OF_INTEREST_INCLUDE_DIR OBJECTS_OF_INTEREST_LIBRARY
  VEHICLE_INFO_UTILS_INCLUDE_DIR VEHICLE_INFO_UTILS_LIBRARY
  UNIVERSE_UTILS_INCLUDE_DIR)
  set(${provider} "" CACHE PATH "staged Static Obstacle Avoidance ROS1 provider")
  if(NOT EXISTS "${${provider}}")
    message(FATAL_ERROR "Set ${provider} to its staged ROS1 provider; ambient discovery is forbidden")
  endif()
endforeach()

catkin_package(LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs pluginlib roscpp sensor_msgs visualization_msgs)
include_directories(${BPP_COMMON_INCLUDE_DIR} include src ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
  ${ADAPI_V1_MSGS_INCLUDE_DIR} ${TIER4_PLANNING_MSGS_INCLUDE_DIR} ${BOUNDARY_DEPARTURE_CHECKER_INCLUDE_DIR} ${MAGIC_ENUM_INCLUDE_DIR}
  ${FREESPACE_ALGORITHMS_INCLUDE_DIR}
  ${OBJECT_RECOGNITION_UTILS_INCLUDE_DIR}
  ${LANELET2_UTILS_INCLUDE_DIR} ${LANELET2_EXTENSION_INCLUDE_DIR}
  ${MOTION_UTILS_INCLUDE_DIR} ${ROUTE_HANDLER_INCLUDE_DIR} ${RTC_INTERFACE_INCLUDE_DIR}
  ${PLANNING_FACTOR_INTERFACE_INCLUDE_DIR}
  ${OBJECTS_OF_INTEREST_INCLUDE_DIR} ${VEHICLE_INFO_UTILS_INCLUDE_DIR} ${UNIVERSE_UTILS_INCLUDE_DIR})

add_library(${PROJECT_NAME} SHARED
  src/scene.cpp
  src/debug.cpp
  src/utils.cpp
  src/manager.cpp
  src/shift_line_generator.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
if(AW149_BPP_HOST_ABI)
  target_compile_definitions(${PROJECT_NAME} PRIVATE AW149_BPP_HOST_ABI=1)
endif()
target_link_libraries(${PROJECT_NAME}
  ${BPP_COMMON_LIBRARY}
  # BPP's ELF dependency set is intentionally not relied on as a link
  # interface: GNU ld --as-needed does not resolve its unresolved API calls
  # through that set for this candidate consumer.  Keep the three staged
  # providers explicit, so both this module and its algorithm test close over
  # the same candidate-only ABI.
  ${BOUNDARY_DEPARTURE_CHECKER_LIBRARY} ${BOUNDARY_DEPARTURE_CHECKER_UTILS_LIBRARY}
  ${AUTOWARE_UTILS_DEBUG_LIBRARY}
  ${OBJECT_RECOGNITION_UTILS_LIBRARY} ${TRAFFIC_LIGHT_UTILS_LIBRARY}
  ${LANELET2_UTILS_LIBRARY} ${LANELET2_EXTENSION_LIBRARY}
  ${MOTION_UTILS_LIBRARY} ${ROUTE_HANDLER_LIBRARY} ${RTC_INTERFACE_LIBRARY}
  ${PLANNING_FACTOR_INTERFACE_LIBRARY}
  ${OBJECTS_OF_INTEREST_LIBRARY} ${VEHICLE_INFO_UTILS_LIBRARY} fmt::fmt
  ${lanelet2_core_LIBRARIES} ${lanelet2_routing_LIBRARIES} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(autoware_test_utils REQUIRED)
  # The pure algorithm suite is independently runnable with the staged BPP
  # ABI.  The two node-ingress sources below require the separate
  # autoware_behavior_path_planner test harness, which has no candidate ROS1
  # provider yet; leave them as an explicit later runtime gate rather than
  # importing an original/ambient test workspace.
  catkin_add_gmock(test_static_obstacle_utils
    test/test_utils.cpp
    test/ros1_gtest_main.cpp
  )
  if(TARGET test_static_obstacle_utils)
    target_compile_definitions(test_static_obstacle_utils PRIVATE AW149_BPP_HOST_ABI=1)
    target_link_libraries(test_static_obstacle_utils
      ${PROJECT_NAME}
      ${BOUNDARY_DEPARTURE_CHECKER_LIBRARY} ${BOUNDARY_DEPARTURE_CHECKER_UTILS_LIBRARY}
      ${AUTOWARE_UTILS_DEBUG_LIBRARY}
      ${OBJECT_RECOGNITION_UTILS_LIBRARY} ${TRAFFIC_LIGHT_UTILS_LIBRARY}
      ${catkin_LIBRARIES} ${autoware_test_utils_LIBRARIES})
    target_include_directories(test_static_obstacle_utils PRIVATE src ${autoware_test_utils_INCLUDE_DIRS})
  endif()
  catkin_add_gtest(test_static_obstacle_plugin_discovery
    test/test_static_obstacle_plugin_discovery.cpp
  )
  if(TARGET test_static_obstacle_plugin_discovery)
    target_compile_definitions(test_static_obstacle_plugin_discovery PRIVATE AW149_BPP_HOST_ABI=1)
    target_link_libraries(test_static_obstacle_plugin_discovery
      ${PROJECT_NAME}
      ${BOUNDARY_DEPARTURE_CHECKER_LIBRARY} ${BOUNDARY_DEPARTURE_CHECKER_UTILS_LIBRARY}
      ${AUTOWARE_UTILS_DEBUG_LIBRARY}
      ${OBJECT_RECOGNITION_UTILS_LIBRARY} ${TRAFFIC_LIGHT_UTILS_LIBRARY}
      ${PLANNING_FACTOR_INTERFACE_LIBRARY}
      ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_static_obstacle_manager_init
    test/test_static_obstacle_manager_init.cpp
  )
  if(TARGET test_static_obstacle_manager_init)
    target_compile_definitions(test_static_obstacle_manager_init PRIVATE AW149_BPP_HOST_ABI=1)
    target_link_libraries(test_static_obstacle_manager_init
      ${PROJECT_NAME}
      ${BOUNDARY_DEPARTURE_CHECKER_LIBRARY} ${BOUNDARY_DEPARTURE_CHECKER_UTILS_LIBRARY}
      ${AUTOWARE_UTILS_DEBUG_LIBRARY}
      ${OBJECT_RECOGNITION_UTILS_LIBRARY} ${TRAFFIC_LIGHT_UTILS_LIBRARY}
      ${catkin_LIBRARIES})
  endif()
  # These test executables are part of this package's existing install set;
  # catkin gtests are otherwise excluded from the default build.
  add_custom_target(aw149_static_export_tests ALL DEPENDS
    test_static_obstacle_utils test_static_obstacle_plugin_discovery
    test_static_obstacle_manager_init)
  install(TARGETS
    test_static_obstacle_utils
    test_static_obstacle_plugin_discovery
    test_static_obstacle_manager_init
    RUNTIME DESTINATION lib/${PROJECT_NAME})
  message(STATUS "Static Obstacle node-ingress tests await candidate autoware_behavior_path_planner test harness")
endif()

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION include)
# Static Obstacle's exported headers intentionally retain a small ROS2-name
# compatibility surface (rclcpp::Time, Logger and Parameter).  This facade is
# part of the ROS1 candidate ABI, so install it with the public headers rather
# than relying on this package's private src/ include path in downstream
# consumers.
install(DIRECTORY src/rclcpp/ DESTINATION include/rclcpp FILES_MATCHING PATTERN "*.hpp")
install(FILES package.xml plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config test_data DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

find_package(autoware_utils_rclcpp REQUIRED)
include_directories(BEFORE ${autoware_utils_rclcpp_INCLUDE_DIRS})
# This package is also present in install_isolated while being rebuilt. Keep its current source
# headers ahead of that installed copy so incremental builds cannot compile new sources against a
# stale self-header.
include_directories(BEFORE include src)

# Installed test executables require their catkin-built GoogleTest library.
# This changes only installation closure, not planner or test behavior.
if(CATKIN_ENABLE_TESTING AND TARGET gtest)
  install(TARGETS gtest
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
  install(FILES "/usr/share/doc/libgtest-dev/copyright"
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/third_party
    RENAME gtest-LICENSE)
endif()
