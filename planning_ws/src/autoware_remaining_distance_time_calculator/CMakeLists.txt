cmake_minimum_required(VERSION 3.0.2)
project(autoware_remaining_distance_time_calculator)

unset(autoware_test_utils_DIR CACHE)
unset(autoware_internal_msgs_DIR CACHE)
unset(autoware_internal_planning_msgs_DIR CACHE)
unset(autoware_lanelet2_utils_DIR CACHE)
unset(autoware_map_msgs_DIR CACHE)
unset(autoware_planning_msgs_DIR CACHE)
unset(autoware_route_handler_DIR CACHE)
find_package(autoware_test_utils REQUIRED CONFIG)
find_package(catkin REQUIRED COMPONENTS geometry_msgs nav_msgs roscpp)
find_package(autoware_internal_msgs REQUIRED)
find_package(autoware_internal_planning_msgs REQUIRED)
find_package(autoware_lanelet2_utils REQUIRED)
find_package(autoware_map_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(autoware_route_handler REQUIRED)

catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp)
include_directories(include ${catkin_INCLUDE_DIRS}
  ${autoware_internal_msgs_INCLUDE_DIRS} ${autoware_internal_planning_msgs_INCLUDE_DIRS}
  ${autoware_lanelet2_utils_INCLUDE_DIRS} ${autoware_map_msgs_INCLUDE_DIRS}
  ${autoware_planning_msgs_INCLUDE_DIRS} ${autoware_route_handler_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED src/remaining_distance_time_calculator_node.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}
  ${autoware_lanelet2_utils_LIBRARIES} ${autoware_route_handler_LIBRARIES})
add_executable(autoware_remaining_distance_time_calculator_node src/main.cpp)
target_link_libraries(autoware_remaining_distance_time_calculator_node ${PROJECT_NAME})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_remaining_distance_time_transport test/test_remaining_distance_time_transport.cpp)
  if(TARGET test_remaining_distance_time_transport)
    target_compile_definitions(test_remaining_distance_time_transport PRIVATE
      AW149_ROUTE_FIXTURE_PATH="${autoware_test_utils_DIR}/../test_map/overlap_map.osm")
    target_link_libraries(test_remaining_distance_time_transport ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS ${PROJECT_NAME} autoware_remaining_distance_time_calculator_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.yaml" PATTERN "*.launch")
install(FILES package.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
