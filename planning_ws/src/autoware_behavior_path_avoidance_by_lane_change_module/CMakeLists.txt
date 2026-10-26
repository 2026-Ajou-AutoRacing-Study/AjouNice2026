cmake_minimum_required(VERSION 3.0.2)
project(autoware_behavior_path_avoidance_by_lane_change_module)

# Candidate-only ROS1 build boundary.  These providers are deliberately
# explicit: the module must not fall back to an ambient ROS2/AjouNice tree.
find_package(catkin REQUIRED COMPONENTS autoware_frenet_planner autoware_map_msgs autoware_object_recognition_utils autoware_objects_of_interest_marker_interface autoware_planning_factor_interface geometry_msgs pluginlib roscpp tier4_planning_msgs visualization_msgs)
find_package(Eigen3 REQUIRED)
include("${CMAKE_CURRENT_LIST_DIR}/../cmake/AW149Providers.cmake")
aw149_provider(AUTOWARE_UTILS_DEBUG autoware_utils_debug autoware_utils_debug)
aw149_provider(BOUNDARY_DEPARTURE_CHECKER autoware_deprecated_boundary_departure_checker boundary_departure_checker)
aw149_provider(BOUNDARY_DEPARTURE_CHECKER_UTILS autoware_deprecated_boundary_departure_checker boundary_departure_checker_utils)
aw149_provider(BPP_COMMON autoware_behavior_path_planner_common autoware_behavior_path_planner_common)
aw149_provider(LANELET2_UTILS autoware_lanelet2_utils autoware_lanelet2_utils)
aw149_provider(LANE_CHANGE autoware_behavior_path_lane_change_module autoware_behavior_path_lane_change_module)
aw149_provider(LANE_CHANGE_SOURCE autoware_behavior_path_lane_change_module NONE)
aw149_provider(MOTION_UTILS autoware_motion_utils autoware_motion_utils)
aw149_provider(ROUTE_HANDLER autoware_route_handler autoware_route_handler)
aw149_provider(RTC_INTERFACE autoware_rtc_interface autoware_rtc_interface)
aw149_provider(STATIC_AVOIDANCE autoware_behavior_path_static_obstacle_avoidance_module autoware_behavior_path_static_obstacle_avoidance_module)
aw149_provider(TRAFFIC_LIGHT_UTILS autoware_traffic_light_utils autoware_traffic_light_utils)
aw149_provider(UNIVERSE_UTILS autoware_universe_utils NONE)
aw149_provider(VEHICLE_INFO_UTILS autoware_vehicle_info_utils vehicle_info_utils)
set(BPP_COMMON_INCLUDE_DIR "" CACHE PATH "staged BPP common include")
set(BPP_COMMON_LIBRARY "" CACHE FILEPATH "staged BPP common library")
set(LANE_CHANGE_INCLUDE_DIR "" CACHE PATH "staged lane-change include")
set(LANE_CHANGE_LIBRARY "" CACHE FILEPATH "staged lane-change library")
set(STATIC_AVOIDANCE_INCLUDE_DIR "" CACHE PATH "staged static-avoidance include")
set(STATIC_AVOIDANCE_LIBRARY "" CACHE FILEPATH "staged static-avoidance library")
set(MOTION_UTILS_INCLUDE_DIR "" CACHE PATH "staged motion-utils include")
set(MOTION_UTILS_LIBRARY "" CACHE FILEPATH "staged motion-utils library")
set(RTC_INTERFACE_INCLUDE_DIR "" CACHE PATH "staged RTC include")
set(RTC_INTERFACE_LIBRARY "" CACHE FILEPATH "staged RTC library")
set(UNIVERSE_UTILS_INCLUDE_DIR "" CACHE PATH "staged universe-utils include")
set(VEHICLE_INFO_UTILS_INCLUDE_DIR "" CACHE PATH "staged vehicle-info-utils include required by BPP Common public headers")
set(VEHICLE_INFO_UTILS_LIBRARY "" CACHE FILEPATH "staged vehicle-info-utils library")
set(ROUTE_HANDLER_INCLUDE_DIR "" CACHE PATH "staged route-handler include required by LaneChange public headers")
set(ROUTE_HANDLER_LIBRARY "" CACHE FILEPATH "staged route-handler library")
set(LANELET2_UTILS_INCLUDE_DIR "" CACHE PATH "staged lanelet2-utils include required by BPP Common public headers")
set(LANELET2_UTILS_LIBRARY "" CACHE FILEPATH "staged lanelet2-utils library")
set(BOUNDARY_DEPARTURE_CHECKER_LIBRARY "" CACHE FILEPATH "staged boundary-departure provider required by BPP Common ABI")
set(BOUNDARY_DEPARTURE_CHECKER_UTILS_LIBRARY "" CACHE FILEPATH "staged boundary-departure utilities required by BPP Common ABI")
set(TRAFFIC_LIGHT_UTILS_LIBRARY "" CACHE FILEPATH "staged traffic-light provider required by BPP Common ABI")
set(AUTOWARE_UTILS_DEBUG_LIBRARY "" CACHE FILEPATH "staged debug provider required by LaneChange ABI")
set(LANE_CHANGE_SOURCE_INCLUDE_DIR "" CACHE PATH "lane-change source include for its vendored magic_enum header")
set(LANE_CHANGE_PARAMETER_LOADER "" CACHE FILEPATH "candidate-local ROS2-to-ROS1 lane-change parameter adapter")
if(NOT LANE_CHANGE_PARAMETER_LOADER)
  get_filename_component(AW149_LANE_CHANGE_PREFIX "${autoware_behavior_path_lane_change_module_DIR}/../../.." ABSOLUTE)
  set(LANE_CHANGE_PARAMETER_LOADER "${AW149_LANE_CHANGE_PREFIX}/lib/autoware_behavior_path_lane_change_module/load_ros1_parameters.py")
endif()
foreach(provider IN ITEMS BPP_COMMON_INCLUDE_DIR BPP_COMMON_LIBRARY LANE_CHANGE_INCLUDE_DIR LANE_CHANGE_LIBRARY STATIC_AVOIDANCE_INCLUDE_DIR STATIC_AVOIDANCE_LIBRARY MOTION_UTILS_INCLUDE_DIR MOTION_UTILS_LIBRARY RTC_INTERFACE_INCLUDE_DIR RTC_INTERFACE_LIBRARY UNIVERSE_UTILS_INCLUDE_DIR VEHICLE_INFO_UTILS_INCLUDE_DIR VEHICLE_INFO_UTILS_LIBRARY ROUTE_HANDLER_INCLUDE_DIR ROUTE_HANDLER_LIBRARY LANELET2_UTILS_INCLUDE_DIR LANELET2_UTILS_LIBRARY BOUNDARY_DEPARTURE_CHECKER_LIBRARY BOUNDARY_DEPARTURE_CHECKER_UTILS_LIBRARY TRAFFIC_LIGHT_UTILS_LIBRARY AUTOWARE_UTILS_DEBUG_LIBRARY LANE_CHANGE_SOURCE_INCLUDE_DIR LANE_CHANGE_PARAMETER_LOADER)
  if(NOT EXISTS "${${provider}}")
    message(FATAL_ERROR "Set ${provider} to its staged ROS1 provider; ambient discovery is forbidden")
  endif()
endforeach()

catkin_package(LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS autoware_frenet_planner autoware_map_msgs autoware_object_recognition_utils autoware_objects_of_interest_marker_interface autoware_planning_factor_interface geometry_msgs pluginlib roscpp tier4_planning_msgs visualization_msgs)
include_directories(include src ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${BPP_COMMON_INCLUDE_DIR} ${LANE_CHANGE_INCLUDE_DIR} ${LANE_CHANGE_SOURCE_INCLUDE_DIR} ${STATIC_AVOIDANCE_INCLUDE_DIR} ${MOTION_UTILS_INCLUDE_DIR} ${RTC_INTERFACE_INCLUDE_DIR} ${UNIVERSE_UTILS_INCLUDE_DIR} ${VEHICLE_INFO_UTILS_INCLUDE_DIR} ${ROUTE_HANDLER_INCLUDE_DIR} ${LANELET2_UTILS_INCLUDE_DIR})
add_library(${PROJECT_NAME} SHARED src/scene.cpp src/manager.cpp src/interface.cpp src/factory.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_definitions(${PROJECT_NAME} PUBLIC AW149_BPP_HOST_ABI=1)
target_link_libraries(${PROJECT_NAME} ${BPP_COMMON_LIBRARY} ${LANE_CHANGE_LIBRARY} ${STATIC_AVOIDANCE_LIBRARY} ${MOTION_UTILS_LIBRARY} ${RTC_INTERFACE_LIBRARY} ${VEHICLE_INFO_UTILS_LIBRARY} ${ROUTE_HANDLER_LIBRARY} ${LANELET2_UTILS_LIBRARY} ${BOUNDARY_DEPARTURE_CHECKER_LIBRARY} ${BOUNDARY_DEPARTURE_CHECKER_UTILS_LIBRARY} ${TRAFFIC_LIGHT_UTILS_LIBRARY} ${AUTOWARE_UTILS_DEBUG_LIBRARY} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  # The upstream test imports the ROS2 behavior-path-planner harness.  Keep it
  # out of this candidate closure and verify the ROS1 plugin descriptor with
  # the real SceneModuleManagerInterface instead.
  catkin_add_gtest(test_avoidance_by_lane_change_plugin_discovery
    test/test_avoidance_by_lane_change_plugin_discovery.cpp)
  if(TARGET test_avoidance_by_lane_change_plugin_discovery)
    target_link_libraries(test_avoidance_by_lane_change_plugin_discovery
      ${PROJECT_NAME} ${BPP_COMMON_LIBRARY} ${LANE_CHANGE_LIBRARY} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_avoidance_by_lane_change_manager_init
    test/test_avoidance_by_lane_change_manager_init.cpp)
  if(TARGET test_avoidance_by_lane_change_manager_init)
    target_include_directories(test_avoidance_by_lane_change_manager_init PRIVATE src)
    target_compile_definitions(test_avoidance_by_lane_change_manager_init PRIVATE
      LANE_CHANGE_PARAMETER_LOADER="${LANE_CHANGE_PARAMETER_LOADER}")
    target_link_libraries(test_avoidance_by_lane_change_manager_init
      ${PROJECT_NAME} ${BPP_COMMON_LIBRARY} ${LANE_CHANGE_LIBRARY} ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  install(TARGETS test_avoidance_by_lane_change_manager_init
    RUNTIME DESTINATION lib/${PROJECT_NAME})
  add_custom_target(aw149_avoidance_lane_change_export_test ALL
    DEPENDS test_avoidance_by_lane_change_manager_init)
endif()
install(FILES package.xml plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config test DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

find_package(autoware_utils_rclcpp REQUIRED)
include_directories(BEFORE ${autoware_utils_rclcpp_INCLUDE_DIRS})

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

# Keep provider headers ahead of the merged install include tree (also added by
# transitive dependencies). Mixing installed LC layouts with a candidate DSO is invalid.
target_include_directories(autoware_behavior_path_avoidance_by_lane_change_module BEFORE PRIVATE ${LANE_CHANGE_INCLUDE_DIR})
