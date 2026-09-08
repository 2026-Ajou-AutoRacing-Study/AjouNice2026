cmake_minimum_required(VERSION 3.0.2)
project(autoware_behavior_path_external_request_lane_change_module)

find_package(catkin REQUIRED COMPONENTS pluginlib roscpp)
find_package(Eigen3 REQUIRED)
include("${CMAKE_CURRENT_LIST_DIR}/../cmake/AW149Providers.cmake")
aw149_provider(BPP_COMMON autoware_behavior_path_planner_common autoware_behavior_path_planner_common)
aw149_provider(BPP_COMMON_UTILS autoware_behavior_path_planner_common NONE)
aw149_provider(FRENET_PLANNER autoware_frenet_planner autoware_frenet_planner)
aw149_provider(LANE_CHANGE autoware_behavior_path_lane_change_module autoware_behavior_path_lane_change_module)
aw149_provider(MAP_MSGS autoware_map_msgs NONE)
aw149_provider(OBJECTS_OF_INTEREST autoware_objects_of_interest_marker_interface autoware_objects_of_interest_marker_interface)
aw149_provider(PERCEPTION_MSGS autoware_perception_msgs NONE)
aw149_provider(PLANNING_FACTOR autoware_planning_factor_interface autoware_planning_factor_interface)
aw149_provider(ROUTE_HANDLER autoware_route_handler autoware_route_handler)
aw149_provider(RTC_INTERFACE autoware_rtc_interface autoware_rtc_interface)
aw149_provider(SAMPLER_COMMON autoware_sampler_common autoware_sampler_common)
aw149_provider(TIER4_PLANNING_MSGS tier4_planning_msgs NONE)
aw149_provider(UNIVERSE_UTILS autoware_universe_utils NONE)
aw149_provider(VEHICLE_INFO_UTILS autoware_vehicle_info_utils vehicle_info_utils)
set(LANE_CHANGE_INCLUDE_DIR "" CACHE PATH "staged lane-change include")
set(LANE_CHANGE_LIBRARY "" CACHE FILEPATH "staged lane-change library")
set(BPP_COMMON_INCLUDE_DIR "" CACHE PATH "staged BPP common include")
set(BPP_COMMON_LIBRARY "" CACHE FILEPATH "staged BPP common library")
set(PERCEPTION_MSGS_INCLUDE_DIR "" CACHE PATH "staged perception messages include")
set(UNIVERSE_UTILS_INCLUDE_DIR "" CACHE PATH "staged universe-utils include")
set(VEHICLE_INFO_UTILS_INCLUDE_DIR "" CACHE PATH "staged vehicle-info include")
set(ROUTE_HANDLER_INCLUDE_DIR "" CACHE PATH "staged route-handler include")
set(MAP_MSGS_INCLUDE_DIR "" CACHE PATH "staged autoware_map_msgs include")
set(TIER4_PLANNING_MSGS_INCLUDE_DIR "" CACHE PATH "staged tier4_planning_msgs include")
set(FRENET_PLANNER_INCLUDE_DIR "" CACHE PATH "staged autoware_frenet_planner include")
set(SAMPLER_COMMON_INCLUDE_DIR "" CACHE PATH "staged autoware_sampler_common include")
set(OBJECTS_OF_INTEREST_INCLUDE_DIR "" CACHE PATH "staged objects-of-interest marker interface include")
set(BPP_COMMON_UTILS_INCLUDE_DIR "" CACHE PATH "staged BPP common utility headers")
set(PLANNING_FACTOR_INCLUDE_DIR "" CACHE PATH "staged planning-factor interface include")
set(RTC_INTERFACE_INCLUDE_DIR "" CACHE PATH "staged RTC interface include")
foreach(provider IN ITEMS LANE_CHANGE_INCLUDE_DIR LANE_CHANGE_LIBRARY BPP_COMMON_INCLUDE_DIR BPP_COMMON_LIBRARY PERCEPTION_MSGS_INCLUDE_DIR UNIVERSE_UTILS_INCLUDE_DIR VEHICLE_INFO_UTILS_INCLUDE_DIR ROUTE_HANDLER_INCLUDE_DIR MAP_MSGS_INCLUDE_DIR TIER4_PLANNING_MSGS_INCLUDE_DIR FRENET_PLANNER_INCLUDE_DIR SAMPLER_COMMON_INCLUDE_DIR OBJECTS_OF_INTEREST_INCLUDE_DIR BPP_COMMON_UTILS_INCLUDE_DIR PLANNING_FACTOR_INCLUDE_DIR RTC_INTERFACE_INCLUDE_DIR)
  if(NOT EXISTS "${${provider}}")
    message(FATAL_ERROR "Set ${provider} to its staged ROS1 provider; ambient discovery is forbidden")
  endif()
endforeach()

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS pluginlib roscpp
)

add_library(${PROJECT_NAME} SHARED
  src/manager.cpp
  src/scene.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PRIVATE src
  SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${LANE_CHANGE_INCLUDE_DIR}
    ${BPP_COMMON_INCLUDE_DIR}
    ${PERCEPTION_MSGS_INCLUDE_DIR}
    ${UNIVERSE_UTILS_INCLUDE_DIR}
    ${VEHICLE_INFO_UTILS_INCLUDE_DIR}
    ${ROUTE_HANDLER_INCLUDE_DIR}
    ${MAP_MSGS_INCLUDE_DIR}
    ${TIER4_PLANNING_MSGS_INCLUDE_DIR}
    ${FRENET_PLANNER_INCLUDE_DIR}
    ${SAMPLER_COMMON_INCLUDE_DIR}
    ${OBJECTS_OF_INTEREST_INCLUDE_DIR}
    ${BPP_COMMON_UTILS_INCLUDE_DIR}
    ${PLANNING_FACTOR_INCLUDE_DIR}
    ${RTC_INTERFACE_INCLUDE_DIR}
)
target_link_libraries(${PROJECT_NAME}
  ${LANE_CHANGE_LIBRARY}
  ${BPP_COMMON_LIBRARY}
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_external_request_lane_change_plugins
    test/test_external_request_lane_change_plugins.cpp
  )
  if(TARGET test_external_request_lane_change_plugins)
    target_link_libraries(test_external_request_lane_change_plugins
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
      gtest_main
    )
  endif()
endif()

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(FILES package.xml plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
