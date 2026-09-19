cmake_minimum_required(VERSION 3.0.2)
project(autoware_manual_lane_change_handler)

find_package(catkin REQUIRED COMPONENTS
  autoware_common_msgs autoware_internal_debug_msgs autoware_map_msgs
  autoware_planning_msgs geometry_msgs nav_msgs roscpp std_msgs
  tier4_external_api_msgs tier4_planning_msgs)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

include("${CMAKE_CURRENT_LIST_DIR}/../cmake/AW149Providers.cmake")
aw149_provider(ROUTE_HANDLER autoware_route_handler autoware_route_handler)
aw149_provider(LANELET_UTILS autoware_lanelet2_utils autoware_lanelet2_utils)
aw149_provider(LANELET_EXTENSION autoware_lanelet2_extension autoware_lanelet2_extension_lib)
aw149_provider(GEOMETRY autoware_utils_geometry autoware_utils_geometry)
set(ROUTE_HANDLER_INCLUDE_DIR "" CACHE PATH "installed ROS1 route-handler include directory")
set(ROUTE_HANDLER_LIBRARY "" CACHE FILEPATH "installed ROS1 route-handler library")
set(LANELET_PROVIDER_LIBRARY_DIR "" CACHE PATH "installed ROS1 lanelet provider library directory")
set(LANELET2_LIBRARY_DIR "/opt/ros/noetic/lib/x86_64-linux-gnu" CACHE PATH "system ROS1 lanelet2 library directory")
foreach(v ROUTE_HANDLER_INCLUDE_DIR ROUTE_HANDLER_LIBRARY)
  if(NOT EXISTS "${${v}}")
    message(FATAL_ERROR "Set ${v} to the staged ROS1 route-handler provider")
  endif()
endforeach()

catkin_package(CATKIN_DEPENDS autoware_common_msgs autoware_internal_debug_msgs autoware_map_msgs autoware_planning_msgs geometry_msgs nav_msgs roscpp std_msgs tier4_external_api_msgs tier4_planning_msgs)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${ROUTE_HANDLER_INCLUDE_DIR})
add_executable(${PROJECT_NAME}_node src/manual_lane_change_handler.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${ROUTE_HANDLER_LIBRARY}
  ${LANELET_UTILS_LIBRARY}
  ${LANELET_EXTENSION_LIBRARY}
  ${GEOMETRY_LIBRARY}
  ${LANELET2_LIBRARY_DIR}/liblanelet2_core.so
  ${LANELET2_LIBRARY_DIR}/liblanelet2_routing.so
  ${LANELET2_LIBRARY_DIR}/liblanelet2_traffic_rules.so
  ${catkin_LIBRARIES})
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)
install(TARGETS ${PROJECT_NAME}_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
