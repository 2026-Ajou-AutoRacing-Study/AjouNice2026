cmake_minimum_required(VERSION 3.0.2)
project(autoware_behavior_path_lane_change_module)

find_package(catkin REQUIRED COMPONENTS geometry_msgs pluginlib roscpp std_msgs visualization_msgs)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(lanelet2_routing REQUIRED)
find_package(lanelet2_core REQUIRED)
set(BPP_COMMON_INCLUDE_DIR "" CACHE PATH "staged BPP common include")
set(BPP_COMMON_LIBRARY "" CACHE FILEPATH "staged BPP common library")
set(FRENET_PLANNER_INCLUDE_DIR "" CACHE PATH "staged Frenet planner include")
set(FRENET_PLANNER_LIBRARY "" CACHE FILEPATH "staged Frenet planner library")
set(LANELET2_UTILS_INCLUDE_DIR "" CACHE PATH "staged lanelet2-utils include")
set(LANELET2_UTILS_LIBRARY "" CACHE FILEPATH "staged lanelet2-utils library")
set(LANELET2_EXTENSION_INCLUDE_DIR "" CACHE PATH "staged lanelet2 extension include")
set(LANELET2_EXTENSION_LIBRARY "" CACHE FILEPATH "staged lanelet2 extension library")
set(MOTION_UTILS_INCLUDE_DIR "" CACHE PATH "staged motion-utils include")
set(MOTION_UTILS_LIBRARY "" CACHE FILEPATH "staged motion-utils library")
set(INTERPOLATION_LIBRARY "" CACHE FILEPATH "staged interpolation library")
set(GEOMETRY_UTILS_LIBRARY "" CACHE FILEPATH "staged geometry-utils library")
set(DEBUG_UTILS_LIBRARY "" CACHE FILEPATH "staged debug-utils library")
set(PLANNING_FACTOR_INTERFACE_LIBRARY "" CACHE FILEPATH "staged planning-factor interface library")
set(TRAFFIC_LIGHT_UTILS_LIBRARY "" CACHE FILEPATH "staged traffic-light utils library")
set(TEST_UTILS_INCLUDE_DIR "" CACHE PATH "staged ROS1 autoware-test-utils include")
set(TEST_UTILS_LIBRARY "" CACHE FILEPATH "staged ROS1 autoware-test-utils library")
set(RTC_INTERFACE_INCLUDE_DIR "" CACHE PATH "staged RTC include")
set(RTC_INTERFACE_LIBRARY "" CACHE FILEPATH "staged RTC library")
set(UNIVERSE_UTILS_INCLUDE_DIR "" CACHE PATH "staged universe-utils include")
set(VEHICLE_INFO_UTILS_INCLUDE_DIR "" CACHE PATH "staged vehicle-info include")
set(VEHICLE_INFO_UTILS_LIBRARY "" CACHE FILEPATH "staged vehicle-info library")
set(ROUTE_HANDLER_INCLUDE_DIR "" CACHE PATH "staged route-handler include")
set(ROUTE_HANDLER_LIBRARY "" CACHE FILEPATH "staged route-handler library")
set(SAMPLER_COMMON_INCLUDE_DIR "" CACHE PATH "staged sampler-common include")
set(SAMPLER_COMMON_LIBRARY "" CACHE FILEPATH "staged sampler-common library")
set(ADAPI_V1_MSGS_INCLUDE_DIR "" CACHE PATH "staged ADAPI v1 messages include")
set(INTERNAL_PLANNING_MSGS_INCLUDE_DIR "" CACHE PATH "staged internal planning messages include")
set(TIER4_PLANNING_MSGS_INCLUDE_DIR "" CACHE PATH "staged tier4 planning messages include")
set(OBJECTS_OF_INTEREST_INCLUDE_DIR "" CACHE PATH "staged objects-of-interest marker include")
set(OBJECTS_OF_INTEREST_LIBRARY "" CACHE FILEPATH "staged objects-of-interest marker library")
set(OBJECT_RECOGNITION_UTILS_INCLUDE_DIR "" CACHE PATH "staged object-recognition utils include")
set(OBJECT_RECOGNITION_UTILS_LIBRARY "" CACHE FILEPATH "staged object-recognition utils library")
set(BOUNDARY_DEPARTURE_CHECKER_UTILS_LIBRARY "" CACHE FILEPATH "staged boundary-departure checker utils library required by BPP Common R025")
include("${CMAKE_CURRENT_LIST_DIR}/../cmake/AW149Providers.cmake")
aw149_provider(BPP_COMMON autoware_behavior_path_planner_common autoware_behavior_path_planner_common)
aw149_provider(VEHICLE_INFO_UTILS autoware_vehicle_info_utils vehicle_info_utils)
aw149_provider(ROUTE_HANDLER autoware_route_handler autoware_route_handler)
aw149_provider(LANELET2_UTILS autoware_lanelet2_utils autoware_lanelet2_utils)
aw149_provider(MOTION_UTILS autoware_motion_utils autoware_motion_utils)
aw149_provider(INTERPOLATION autoware_interpolation autoware_interpolation)
aw149_provider(PLANNING_FACTOR_INTERFACE autoware_planning_factor_interface autoware_planning_factor_interface)
aw149_provider(RTC_INTERFACE autoware_rtc_interface autoware_rtc_interface)
aw149_provider(TRAFFIC_LIGHT_UTILS autoware_traffic_light_utils autoware_traffic_light_utils)
aw149_provider(OBJECT_RECOGNITION_UTILS autoware_object_recognition_utils autoware_object_recognition_utils)
aw149_provider(FRENET_PLANNER autoware_frenet_planner autoware_frenet_planner)
aw149_provider(LANELET2_EXTENSION autoware_lanelet2_extension autoware_lanelet2_extension_lib)
aw149_provider(GEOMETRY_UTILS autoware_utils_geometry autoware_utils_geometry)
aw149_provider(DEBUG_UTILS autoware_utils_debug autoware_utils_debug)
aw149_provider(TEST_UTILS autoware_test_utils autoware_test_utils)
aw149_provider(UNIVERSE_UTILS autoware_universe_utils NONE)
aw149_provider(SAMPLER_COMMON autoware_sampler_common autoware_sampler_common)
aw149_provider(ADAPI_V1_MSGS autoware_adapi_v1_msgs NONE)
aw149_provider(INTERNAL_PLANNING_MSGS autoware_internal_planning_msgs NONE)
aw149_provider(TIER4_PLANNING_MSGS tier4_planning_msgs NONE)
aw149_provider(OBJECTS_OF_INTEREST autoware_objects_of_interest_marker_interface autoware_objects_of_interest_marker_interface)
aw149_provider(BOUNDARY_DEPARTURE_CHECKER_UTILS autoware_deprecated_boundary_departure_checker boundary_departure_checker_utils)
foreach(provider IN ITEMS BPP_COMMON_INCLUDE_DIR BPP_COMMON_LIBRARY FRENET_PLANNER_INCLUDE_DIR FRENET_PLANNER_LIBRARY LANELET2_UTILS_INCLUDE_DIR LANELET2_UTILS_LIBRARY LANELET2_EXTENSION_INCLUDE_DIR LANELET2_EXTENSION_LIBRARY MOTION_UTILS_INCLUDE_DIR MOTION_UTILS_LIBRARY INTERPOLATION_LIBRARY GEOMETRY_UTILS_LIBRARY DEBUG_UTILS_LIBRARY PLANNING_FACTOR_INTERFACE_LIBRARY TRAFFIC_LIGHT_UTILS_LIBRARY TEST_UTILS_INCLUDE_DIR TEST_UTILS_LIBRARY RTC_INTERFACE_INCLUDE_DIR RTC_INTERFACE_LIBRARY UNIVERSE_UTILS_INCLUDE_DIR VEHICLE_INFO_UTILS_INCLUDE_DIR VEHICLE_INFO_UTILS_LIBRARY ROUTE_HANDLER_INCLUDE_DIR ROUTE_HANDLER_LIBRARY SAMPLER_COMMON_INCLUDE_DIR SAMPLER_COMMON_LIBRARY ADAPI_V1_MSGS_INCLUDE_DIR INTERNAL_PLANNING_MSGS_INCLUDE_DIR TIER4_PLANNING_MSGS_INCLUDE_DIR OBJECTS_OF_INTEREST_INCLUDE_DIR OBJECTS_OF_INTEREST_LIBRARY OBJECT_RECOGNITION_UTILS_INCLUDE_DIR OBJECT_RECOGNITION_UTILS_LIBRARY BOUNDARY_DEPARTURE_CHECKER_UTILS_LIBRARY)
  if(NOT EXISTS "${${provider}}")
    message(FATAL_ERROR "Set ${provider} to its staged ROS1 provider; ambient discovery is forbidden")
  endif()
endforeach()
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs pluginlib roscpp std_msgs visualization_msgs
)
include_directories(include src ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${BPP_COMMON_INCLUDE_DIR} ${FRENET_PLANNER_INCLUDE_DIR} ${LANELET2_UTILS_INCLUDE_DIR} ${LANELET2_EXTENSION_INCLUDE_DIR} ${MOTION_UTILS_INCLUDE_DIR} ${TEST_UTILS_INCLUDE_DIR} ${RTC_INTERFACE_INCLUDE_DIR} ${UNIVERSE_UTILS_INCLUDE_DIR} ${VEHICLE_INFO_UTILS_INCLUDE_DIR} ${ROUTE_HANDLER_INCLUDE_DIR} ${SAMPLER_COMMON_INCLUDE_DIR} ${ADAPI_V1_MSGS_INCLUDE_DIR} ${INTERNAL_PLANNING_MSGS_INCLUDE_DIR} ${TIER4_PLANNING_MSGS_INCLUDE_DIR} ${OBJECTS_OF_INTEREST_INCLUDE_DIR} ${OBJECT_RECOGNITION_UTILS_INCLUDE_DIR})
add_library(${PROJECT_NAME} SHARED
  src/interface.cpp
  src/manager.cpp
  src/scene.cpp
  src/safety_diagnostics.cpp
  src/utils/calculation.cpp
  src/utils/markers.cpp
  src/utils/utils.cpp
  src/utils/path.cpp
  src/utils/safety_check.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} nlohmann_json::nlohmann_json)
target_link_libraries(${PROJECT_NAME} ${BPP_COMMON_LIBRARY} ${FRENET_PLANNER_LIBRARY} ${LANELET2_UTILS_LIBRARY} ${LANELET2_EXTENSION_LIBRARY} ${MOTION_UTILS_LIBRARY} ${INTERPOLATION_LIBRARY} ${GEOMETRY_UTILS_LIBRARY} ${DEBUG_UTILS_LIBRARY} ${PLANNING_FACTOR_INTERFACE_LIBRARY} ${TRAFFIC_LIGHT_UTILS_LIBRARY} ${RTC_INTERFACE_LIBRARY} ${VEHICLE_INFO_UTILS_LIBRARY} ${ROUTE_HANDLER_LIBRARY} ${SAMPLER_COMMON_LIBRARY} ${OBJECTS_OF_INTEREST_LIBRARY} ${OBJECT_RECOGNITION_UTILS_LIBRARY} ${BOUNDARY_DEPARTURE_CHECKER_UTILS_LIBRARY} fmt::fmt ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  add_executable(lane_change_safety_phase_replay test/replay_safety_phases.cpp)
  target_link_libraries(lane_change_safety_phase_replay ${PROJECT_NAME} nlohmann_json::nlohmann_json)
  catkin_add_gtest(test_lane_change_safety_phases test/test_safety_phases.cpp)
  if(TARGET test_lane_change_safety_phases)
    target_link_libraries(test_lane_change_safety_phases ${PROJECT_NAME} gtest_main)
  endif()
  # Keep the upstream aggregate target intact.  The ROS2 BPP node harness it
  # includes is a separate porting gate; build the source tests that have no
  # such harness dependency as an additional ROS1 oracle instead of dropping
  # them with the aggregate target.
  catkin_add_gtest(test_lane_change_ros1_core_oracles
    test/test_lane_change_utils.cpp
    test/test_lane_change_scene.cpp
  )
  if(TARGET test_lane_change_ros1_core_oracles)
    target_include_directories(test_lane_change_ros1_core_oracles PRIVATE
      src /usr/src/googletest/googlemock/include)
    target_link_libraries(test_lane_change_ros1_core_oracles ${PROJECT_NAME} ${TEST_UTILS_LIBRARY} yaml-cpp ${lanelet2_routing_LIBRARIES} gmock_main)
  endif()
  # Planning-factor retains its upstream aggregate membership below.  It needs
  # the separately ported BPP node test harness, whereas this additive target
  # isolates the source scene fixture and its map/route/object behavior.
  catkin_add_gtest(test_lane_change_scene_ros1_oracle test/test_lane_change_scene.cpp)
  if(TARGET test_lane_change_scene_ros1_oracle)
    target_include_directories(test_lane_change_scene_ros1_oracle PRIVATE
      src /usr/src/googletest/googlemock/include)
    target_link_libraries(test_lane_change_scene_ros1_oracle ${PROJECT_NAME} ${TEST_UTILS_LIBRARY} yaml-cpp ${lanelet2_routing_LIBRARIES} gtest)
  endif()
  catkin_add_gtest(test_lane_change_utils_ros1_oracle test/test_lane_change_utils.cpp)
  if(TARGET test_lane_change_utils_ros1_oracle)
    target_include_directories(test_lane_change_utils_ros1_oracle PRIVATE
      src /usr/src/googletest/googlemock/include)
    target_link_libraries(test_lane_change_utils_ros1_oracle ${PROJECT_NAME} ${ROUTE_HANDLER_LIBRARY} ${BPP_COMMON_LIBRARY} ${lanelet2_core_LIBRARIES} ${catkin_LIBRARIES} gmock_main)
  endif()
  catkin_add_gtest(test_${PROJECT_NAME}
    test/test_behavior_path_planner_node_interface.cpp
    test/test_lane_change_utils.cpp
    test/test_lane_change_scene.cpp
    test/test_planning_factor.cpp
  )

  if(TARGET test_${PROJECT_NAME})
    find_package(autoware_planning_test_manager REQUIRED CONFIG)
    target_link_directories(test_${PROJECT_NAME} PRIVATE ${CMAKE_INSTALL_PREFIX}/lib)
    target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME}
      autoware_behavior_path_planner_full_source ${autoware_planning_test_manager_LIBRARIES} ${TEST_UTILS_LIBRARY} yaml-cpp gmock)
    target_include_directories(test_${PROJECT_NAME} PRIVATE /usr/src/googletest/googlemock/include)
    target_include_directories(test_${PROJECT_NAME} PRIVATE src)
  endif()
endif()

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION include)
# Public lane-change headers retain a small ROS1 logger/time compatibility
# facade under the historical rclcpp include spelling.  Install the facade with
# the public headers so downstream ROS1 consumers never depend on this source
# tree's private include path.
install(DIRECTORY src/rclcpp/ DESTINATION include/rclcpp)
# `base_class.hpp` includes this compatibility header.  Keeping it private to
# `src/` made the installed public headers non-consumable by a clean catkin
# downstream package, so export the exact staged header alongside them.
install(FILES src/magic_enum.hpp DESTINATION include)
catkin_install_python(PROGRAMS scripts/load_ros1_parameters.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES package.xml plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config test_data DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

find_package(autoware_utils_rclcpp REQUIRED)
include_directories(BEFORE ${autoware_utils_rclcpp_INCLUDE_DIRS})
include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR}/include)
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_native_parameter_events test/test_native_parameter_events.cpp)
  if(TARGET test_native_parameter_events)
    target_compile_features(test_native_parameter_events PUBLIC cxx_std_17)
    target_link_libraries(test_native_parameter_events ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
endif()
