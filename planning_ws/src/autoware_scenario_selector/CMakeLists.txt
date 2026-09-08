cmake_minimum_required(VERSION 3.0.2)
project(autoware_scenario_selector)
# Providers come from this workspace and its declared Noetic underlay.
find_package(catkin REQUIRED COMPONENTS geometry_msgs nav_msgs roscpp std_msgs)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(autoware_internal_debug_msgs REQUIRED)
find_package(autoware_internal_planning_msgs REQUIRED)
find_package(autoware_lanelet2_extension REQUIRED)
find_package(autoware_map_msgs REQUIRED)
find_package(autoware_motion_velocity_planner REQUIRED)
find_package(autoware_motion_velocity_planner_common REQUIRED)
find_package(autoware_velocity_smoother REQUIRED)
find_package(autoware_osqp_interface REQUIRED)
find_package(autoware_qp_interface REQUIRED)
find_package(autoware_object_recognition_utils REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(autoware_route_handler REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${autoware_adapi_v1_msgs_INCLUDE_DIRS}
  ${autoware_internal_debug_msgs_INCLUDE_DIRS}
  ${autoware_internal_planning_msgs_INCLUDE_DIRS}
  ${autoware_lanelet2_extension_INCLUDE_DIRS}
  ${autoware_map_msgs_INCLUDE_DIRS}
  ${autoware_motion_velocity_planner_INCLUDE_DIRS}
  ${autoware_motion_velocity_planner_common_INCLUDE_DIRS}
  ${autoware_velocity_smoother_INCLUDE_DIRS}
  ${autoware_osqp_interface_INCLUDE_DIRS}
  ${autoware_qp_interface_INCLUDE_DIRS}
  ${autoware_object_recognition_utils_INCLUDE_DIRS}
  ${autoware_planning_msgs_INCLUDE_DIRS}
  ${autoware_route_handler_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME} SHARED src/scenario_state.cpp src/node.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${autoware_lanelet2_extension_LIBRARIES}
  ${autoware_motion_velocity_planner_LIBRARIES}
  ${autoware_motion_velocity_planner_common_LIBRARIES}
  ${autoware_velocity_smoother_LIBRARIES}
  ${autoware_osqp_interface_LIBRARIES}
  ${autoware_qp_interface_LIBRARIES}
  ${autoware_object_recognition_utils_LIBRARIES}
  ${autoware_route_handler_LIBRARIES}
)
add_executable(autoware_scenario_selector_node src/main.cpp)
target_link_libraries(autoware_scenario_selector_node ${PROJECT_NAME})
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_scenario_state test/test_scenario_state.cpp)
  if(TARGET test_scenario_state)
    target_link_libraries(test_scenario_state ${PROJECT_NAME})
  endif()
  catkin_add_gtest(test_scenario_selector_node_transport test/test_scenario_selector_node_transport.cpp)
  if(TARGET test_scenario_selector_node_transport)
    target_compile_definitions(test_scenario_selector_node_transport PRIVATE AW149_SCENARIO_SELECTOR_TESTING)
    find_package(autoware_test_utils REQUIRED CONFIG)
    target_compile_definitions(test_scenario_selector_node_transport PRIVATE AW149_ROUTE_FIXTURE_PATH="${autoware_test_utils_DIR}/../test_map/overlap_map.osm")
    target_link_libraries(test_scenario_selector_node_transport ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
endif()
install(TARGETS ${PROJECT_NAME} autoware_scenario_selector_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.yaml" PATTERN "*.launch")
install(FILES package.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
