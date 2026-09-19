cmake_minimum_required(VERSION 3.0.2)
project(autoware_behavior_path_bidirectional_traffic_module)

# This is an intentionally narrow candidate build: it proves the converted
# ROS1 parameter contract before the trajectory/scene source is admitted to a
# BPP slot. It must not use ambient overlays to satisfy a provider.
find_package(catkin REQUIRED COMPONENTS geometry_msgs pluginlib roscpp)
find_package(Eigen3 REQUIRED)
find_package(lanelet2_core REQUIRED)

include("${CMAKE_CURRENT_LIST_DIR}/../cmake/AW149Providers.cmake")
aw149_provider(AUTOWARE_INTERFACE_MSGS autoware_planning_msgs NONE)
aw149_provider(AUTOWARE_TRAJECTORY autoware_trajectory autoware_trajectory)
aw149_provider(BOUNDARY_DEPARTURE_CHECKER_UTILS autoware_deprecated_boundary_departure_checker boundary_departure_checker_utils)
aw149_provider(BPP_COMMON autoware_behavior_path_planner_common autoware_behavior_path_planner_common)
aw149_provider(LANELET2_CORE lanelet2_core lanelet2_core)
aw149_provider(LANELET2_EXTENSION autoware_lanelet2_extension autoware_lanelet2_extension_lib)
aw149_provider(LANELET2_IO lanelet2_io lanelet2_io)
aw149_provider(LANELET2_ROUTING lanelet2_routing lanelet2_routing)
aw149_provider(LANELET2_UTILS autoware_lanelet2_utils autoware_lanelet2_utils)
aw149_provider(MOTION_UTILS autoware_motion_utils autoware_motion_utils)
aw149_provider(OBJECTS_OF_INTEREST autoware_objects_of_interest_marker_interface autoware_objects_of_interest_marker_interface)
aw149_provider(OBJECT_RECOGNITION_UTILS autoware_object_recognition_utils autoware_object_recognition_utils)
aw149_provider(PLANNING_FACTOR autoware_planning_factor_interface autoware_planning_factor_interface)
aw149_provider(ROUTE_HANDLER autoware_route_handler autoware_route_handler)
aw149_provider(RTC_INTERFACE autoware_rtc_interface autoware_rtc_interface)
aw149_provider(TIER4_PLANNING_MSGS tier4_planning_msgs NONE)
aw149_provider(TRAFFIC_LIGHT_UTILS autoware_traffic_light_utils autoware_traffic_light_utils)
aw149_provider(UTILS_GEOMETRY autoware_utils_geometry autoware_utils_geometry)
aw149_provider(VEHICLE_INFO_UTILS autoware_vehicle_info_utils vehicle_info_utils)
set(BPP_COMMON_INCLUDE_DIR "" CACHE PATH "staged BPP-common ROS1 include directory")
set(UTILS_GEOMETRY_INCLUDE_DIR "" CACHE PATH "staged autoware-utils-geometry ROS1 include directory")
set(AUTOWARE_INTERFACE_MSGS_INCLUDE_DIR "" CACHE PATH "staged ROS1 Autoware interface-message include directory")
set(VEHICLE_INFO_UTILS_INCLUDE_DIR "" CACHE PATH "staged ROS1 vehicle-info-utils include directory")
set(MOTION_UTILS_INCLUDE_DIR "" CACHE PATH "staged ROS1 motion-utils include directory")
set(TIER4_PLANNING_MSGS_INCLUDE_DIR "" CACHE PATH "staged ROS1 tier4-planning-msgs include directory")
set(UTILS_GEOMETRY_LIBRARY "" CACHE FILEPATH "staged ROS1 autoware-utils-geometry library")
set(BPP_COMMON_LIBRARY "" CACHE FILEPATH "staged ROS1 BPP-common library")
set(MOTION_UTILS_LIBRARY "" CACHE FILEPATH "staged ROS1 motion-utils library")
set(AUTOWARE_TRAJECTORY_LIBRARY "" CACHE FILEPATH "staged ROS1 autoware-trajectory library")
set(ROUTE_HANDLER_LIBRARY "" CACHE FILEPATH "staged ROS1 route-handler library")
set(PLANNING_FACTOR_LIBRARY "" CACHE FILEPATH "staged ROS1 planning-factor interface library")
set(RTC_INTERFACE_LIBRARY "" CACHE FILEPATH "staged ROS1 RTC interface library")
set(OBJECTS_OF_INTEREST_LIBRARY "" CACHE FILEPATH "staged ROS1 objects-of-interest interface library")
set(LANELET2_CORE_LIBRARY "" CACHE FILEPATH "ROS1 lanelet2-core library")
set(LANELET2_UTILS_LIBRARY "" CACHE FILEPATH "staged ROS1 autoware-lanelet2-utils library")
set(LANELET2_EXTENSION_LIBRARY "" CACHE FILEPATH "staged ROS1 autoware-lanelet2-extension library")
set(OBJECT_RECOGNITION_UTILS_LIBRARY "" CACHE FILEPATH "staged ROS1 object-recognition-utils library")
set(TRAFFIC_LIGHT_UTILS_LIBRARY "" CACHE FILEPATH "staged ROS1 traffic-light-utils library")
set(VEHICLE_INFO_UTILS_LIBRARY "" CACHE FILEPATH "staged ROS1 vehicle-info-utils library")
set(LANELET2_IO_LIBRARY "" CACHE FILEPATH "ROS1 lanelet2-io library")
set(LANELET2_ROUTING_LIBRARY "" CACHE FILEPATH "ROS1 lanelet2-routing library")
set(BOUNDARY_DEPARTURE_CHECKER_UTILS_LIBRARY "" CACHE FILEPATH "staged boundary-departure checker utils required by BPP Common R025")
set(OBJECTS_OF_INTEREST_INCLUDE_DIR "" CACHE PATH "staged ROS1 objects-of-interest interface include directory")
set(PLANNING_FACTOR_INCLUDE_DIR "" CACHE PATH "staged ROS1 planning-factor interface include directory")
set(RTC_INTERFACE_INCLUDE_DIR "" CACHE PATH "staged ROS1 RTC interface include directory")
set(AUTOWARE_TRAJECTORY_INCLUDE_DIR "" CACHE PATH "staged ROS1 autoware-trajectory include directory")
set(ROUTE_HANDLER_INCLUDE_DIR "" CACHE PATH "staged ROS1 route-handler include directory")
foreach(provider IN ITEMS BPP_COMMON_INCLUDE_DIR UTILS_GEOMETRY_INCLUDE_DIR AUTOWARE_INTERFACE_MSGS_INCLUDE_DIR VEHICLE_INFO_UTILS_INCLUDE_DIR MOTION_UTILS_INCLUDE_DIR TIER4_PLANNING_MSGS_INCLUDE_DIR OBJECTS_OF_INTEREST_INCLUDE_DIR PLANNING_FACTOR_INCLUDE_DIR RTC_INTERFACE_INCLUDE_DIR AUTOWARE_TRAJECTORY_INCLUDE_DIR ROUTE_HANDLER_INCLUDE_DIR)
  if(NOT EXISTS "${${provider}}")
    message(FATAL_ERROR "Set ${provider} to an explicit staged ROS1 provider")
  endif()
endforeach()
if(NOT EXISTS "${UTILS_GEOMETRY_LIBRARY}")
  message(FATAL_ERROR "Set UTILS_GEOMETRY_LIBRARY to the explicit staged ROS1 geometry library")
endif()
foreach(provider_library IN ITEMS BPP_COMMON_LIBRARY MOTION_UTILS_LIBRARY AUTOWARE_TRAJECTORY_LIBRARY ROUTE_HANDLER_LIBRARY PLANNING_FACTOR_LIBRARY RTC_INTERFACE_LIBRARY OBJECTS_OF_INTEREST_LIBRARY LANELET2_CORE_LIBRARY LANELET2_UTILS_LIBRARY LANELET2_EXTENSION_LIBRARY OBJECT_RECOGNITION_UTILS_LIBRARY TRAFFIC_LIGHT_UTILS_LIBRARY VEHICLE_INFO_UTILS_LIBRARY LANELET2_IO_LIBRARY LANELET2_ROUTING_LIBRARY BOUNDARY_DEPARTURE_CHECKER_UTILS_LIBRARY)
  if(NOT EXISTS "${${provider_library}}")
    message(FATAL_ERROR "Set ${provider_library} to its explicit staged ROS1 library")
  endif()
endforeach()

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
  ${BPP_COMMON_INCLUDE_DIR} ${UTILS_GEOMETRY_INCLUDE_DIR} ${AUTOWARE_INTERFACE_MSGS_INCLUDE_DIR}
  ${VEHICLE_INFO_UTILS_INCLUDE_DIR} ${MOTION_UTILS_INCLUDE_DIR} ${TIER4_PLANNING_MSGS_INCLUDE_DIR}
  ${OBJECTS_OF_INTEREST_INCLUDE_DIR} ${PLANNING_FACTOR_INCLUDE_DIR} ${RTC_INTERFACE_INCLUDE_DIR}
  ${AUTOWARE_TRAJECTORY_INCLUDE_DIR} ${ROUTE_HANDLER_INCLUDE_DIR})

# The non-system Autoware providers remain explicit cache inputs above.  The
# catkin export therefore contains only the ROS1 packages this candidate can
# discover without an ambient overlay.
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs pluginlib roscpp)

add_library(aw149_bidirectional_traffic_parameter_contract SHARED src/parameter.cpp)
target_compile_features(aw149_bidirectional_traffic_parameter_contract PUBLIC cxx_std_17)
target_link_libraries(aw149_bidirectional_traffic_parameter_contract ${catkin_LIBRARIES} ${UTILS_GEOMETRY_LIBRARY})

add_executable(aw149_bidirectional_traffic_parameter_init test/ros1_parameter_init.cpp)
target_compile_features(aw149_bidirectional_traffic_parameter_init PUBLIC cxx_std_17)
target_link_libraries(aw149_bidirectional_traffic_parameter_init
  aw149_bidirectional_traffic_parameter_contract ${catkin_LIBRARIES})

# The manager compile gate checks the translated provider ownership and exact
# SceneModuleManagerCore ABI without claiming the lane-map/trajectory runtime
# sources have already been admitted to BPP.
add_library(aw149_bidirectional_traffic_manager_contract OBJECT src/manager.cpp)
target_compile_features(aw149_bidirectional_traffic_manager_contract PUBLIC cxx_std_17)

# This gate compiles every converted source unit.  It intentionally remains an
# object target until the lane-map/trajectory provider DSOs and the behavioral
# oracle are separately admitted.
add_library(aw149_bidirectional_traffic_scene_contract OBJECT
  src/bidirectional_lanelets.cpp
  src/give_way.cpp
  src/keep_left.cpp
  src/manager.cpp
  src/oncoming_car.cpp
  src/parameter.cpp
  src/scene.cpp
  src/utils.cpp)
target_compile_features(aw149_bidirectional_traffic_scene_contract PUBLIC cxx_std_17)
set_target_properties(aw149_bidirectional_traffic_scene_contract PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(${PROJECT_NAME} SHARED
  $<TARGET_OBJECTS:aw149_bidirectional_traffic_scene_contract>)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${BPP_COMMON_LIBRARY}
  ${BOUNDARY_DEPARTURE_CHECKER_UTILS_LIBRARY}
  ${LANELET2_CORE_LIBRARY}
  ${LANELET2_UTILS_LIBRARY}
  ${LANELET2_EXTENSION_LIBRARY}
  ${OBJECT_RECOGNITION_UTILS_LIBRARY}
  ${TRAFFIC_LIGHT_UTILS_LIBRARY}
  ${VEHICLE_INFO_UTILS_LIBRARY}
  ${LANELET2_IO_LIBRARY}
  ${LANELET2_ROUTING_LIBRARY}
  ${UTILS_GEOMETRY_LIBRARY}
  ${MOTION_UTILS_LIBRARY}
  ${AUTOWARE_TRAJECTORY_LIBRARY}
  ${ROUTE_HANDLER_LIBRARY}
  ${PLANNING_FACTOR_LIBRARY}
  ${RTC_INTERFACE_LIBRARY}
  ${OBJECTS_OF_INTEREST_LIBRARY})
target_link_options(${PROJECT_NAME} PRIVATE -Wl,--no-undefined)

# Keep the evidence-test target spelling stable while exporting the actual
# ROS1 package library name used by pluginlib.
add_library(aw149_bidirectional_traffic_candidate ALIAS ${PROJECT_NAME})

add_executable(aw149_bidirectional_traffic_manager_init test/ros1_manager_init.cpp)
target_compile_features(aw149_bidirectional_traffic_manager_init PUBLIC cxx_std_17)
target_link_libraries(aw149_bidirectional_traffic_manager_init
  aw149_bidirectional_traffic_candidate ${catkin_LIBRARIES})

find_package(GTest REQUIRED)
add_executable(aw149_bidirectional_oncoming_oracle test/ros1_oncoming_oracle.cpp)
target_compile_features(aw149_bidirectional_oncoming_oracle PUBLIC cxx_std_17)
target_compile_definitions(aw149_bidirectional_oncoming_oracle PRIVATE
  AW149_BIDIRECTIONAL_TEST_MAP="${CMAKE_CURRENT_SOURCE_DIR}/test_map/lanelet2_map.osm")
target_link_libraries(aw149_bidirectional_oncoming_oracle
  aw149_bidirectional_traffic_candidate ${catkin_LIBRARIES} GTest::gtest GTest::gtest_main
  /opt/ros/noetic/lib/x86_64-linux-gnu/liblanelet2_projection.so)

add_executable(aw149_bidirectional_scene_path_oracle test/ros1_scene_path_oracle.cpp)
target_compile_features(aw149_bidirectional_scene_path_oracle PUBLIC cxx_std_17)
target_compile_definitions(aw149_bidirectional_scene_path_oracle PRIVATE
  AW149_BIDIRECTIONAL_TEST_MAP="${CMAKE_CURRENT_SOURCE_DIR}/test_map/lanelet2_map.osm")
target_link_libraries(aw149_bidirectional_scene_path_oracle
  aw149_bidirectional_traffic_candidate ${catkin_LIBRARIES} GTest::gtest GTest::gtest_main
  /opt/ros/noetic/lib/x86_64-linux-gnu/liblanelet2_projection.so)

add_executable(aw149_bidirectional_plugin_runtime test/ros1_plugin_runtime.cpp)
target_compile_features(aw149_bidirectional_plugin_runtime PUBLIC cxx_std_17)
target_link_libraries(aw149_bidirectional_plugin_runtime
  aw149_bidirectional_traffic_candidate ${catkin_LIBRARIES} GTest::gtest)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

find_package(autoware_utils_rclcpp REQUIRED)
include_directories(BEFORE ${autoware_utils_rclcpp_INCLUDE_DIRS})
