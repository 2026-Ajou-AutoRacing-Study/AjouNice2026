cmake_minimum_required(VERSION 3.0.2)
project(autoware_behavior_path_planner_common)


find_package(catkin REQUIRED COMPONENTS autoware_adapi_v1_msgs autoware_deprecated_boundary_departure_checker autoware_internal_debug_msgs autoware_internal_planning_msgs autoware_perception_msgs autoware_planning_msgs autoware_traffic_light_utils autoware_vehicle_msgs autoware_motion_utils autoware_vehicle_info_utils autoware_route_handler autoware_lanelet2_utils autoware_lanelet2_extension autoware_freespace_planning_algorithms autoware_object_recognition_utils autoware_planning_factor_interface autoware_objects_of_interest_marker_interface autoware_rtc_interface geometry_msgs nav_msgs roscpp tf2 tf2_geometry_msgs tier4_planning_msgs unique_identifier_msgs autoware_utils visualization_msgs)
find_package(Eigen3 REQUIRED)
find_library(LANELET2_CORE_LIBRARY NAMES lanelet2_core REQUIRED)

set(MOTION_UTILS_INCLUDE_DIR "" CACHE PATH "staged motion-utils include directory")
set(MOTION_UTILS_LIBRARY "" CACHE FILEPATH "staged motion-utils library")
set(INTERPOLATION_INCLUDE_DIR "" CACHE PATH "staged interpolation include directory")
set(INTERPOLATION_LIBRARY "" CACHE FILEPATH "staged interpolation library")
set(UNIVERSE_UTILS_INCLUDE_DIR "" CACHE PATH "staged universe-utils include directory")
set(VEHICLE_INFO_UTILS_INCLUDE_DIR "" CACHE PATH "staged vehicle-info-utils include directory")
set(VEHICLE_INFO_UTILS_LIBRARY "" CACHE FILEPATH "staged vehicle-info-utils library")
set(ROUTE_HANDLER_INCLUDE_DIR "" CACHE PATH "staged route-handler include directory")
set(ROUTE_HANDLER_LIBRARY "" CACHE FILEPATH "staged route-handler library")
set(PLANNING_FACTOR_INTERFACE_INCLUDE_DIR "" CACHE PATH "staged planning-factor-interface include directory")
set(PLANNING_FACTOR_INTERFACE_LIBRARY "" CACHE FILEPATH "staged planning-factor-interface library")
set(OBJECTS_OF_INTEREST_MARKER_INTERFACE_INCLUDE_DIR "" CACHE PATH "staged objects-of-interest-marker-interface include directory")
set(OBJECTS_OF_INTEREST_MARKER_INTERFACE_LIBRARY "" CACHE FILEPATH "staged objects-of-interest-marker-interface library")
set(RTC_INTERFACE_INCLUDE_DIR "" CACHE PATH "staged rtc-interface include directory")
set(RTC_INTERFACE_LIBRARY "" CACHE FILEPATH "staged rtc-interface library")
set(LANELET2_UTILS_INCLUDE_DIR "" CACHE PATH "staged lanelet2-utils include directory")
set(LANELET2_UTILS_LIBRARY "" CACHE FILEPATH "staged lanelet2-utils library")
set(BOUNDARY_DEPARTURE_CHECKER_INCLUDE_DIR "" CACHE PATH "staged boundary-departure-checker include directory")
set(BOUNDARY_DEPARTURE_CHECKER_LIBRARY "" CACHE FILEPATH "staged boundary-departure-checker library")
set(FREESPACE_ALGORITHMS_INCLUDE_DIR "" CACHE PATH "staged freespace algorithms include directory")
set(FREESPACE_ALGORITHMS_LIBRARY "" CACHE FILEPATH "staged freespace algorithms library")
set(OBJECT_RECOGNITION_UTILS_INCLUDE_DIR "" CACHE PATH "staged object-recognition-utils include directory")
set(OBJECT_RECOGNITION_UTILS_LIBRARY "" CACHE FILEPATH "staged object-recognition-utils library")
get_filename_component(
  AW149_LANELET2_EXTENSION_INCLUDE_DIR
  "${autoware_lanelet2_extension_DIR}/../../../include/autoware_lanelet2_extension"
  ABSOLUTE)
foreach(binding IN ITEMS
    "MOTION_UTILS|autoware_motion_utils"
    "INTERPOLATION|autoware_interpolation"
    "UNIVERSE_UTILS|autoware_universe_utils"
    "VEHICLE_INFO_UTILS|autoware_vehicle_info_utils"
    "ROUTE_HANDLER|autoware_route_handler"
    "PLANNING_FACTOR_INTERFACE|autoware_planning_factor_interface"
    "OBJECTS_OF_INTEREST_MARKER_INTERFACE|autoware_objects_of_interest_marker_interface"
    "RTC_INTERFACE|autoware_rtc_interface"
    "LANELET2_UTILS|autoware_lanelet2_utils"
    "BOUNDARY_DEPARTURE_CHECKER|autoware_deprecated_boundary_departure_checker"
    "FREESPACE_ALGORITHMS|autoware_freespace_planning_algorithms"
    "OBJECT_RECOGNITION_UTILS|autoware_object_recognition_utils")
  string(REPLACE "|" ";" fields "${binding}")
  list(GET fields 0 alias)
  list(GET fields 1 provider)
  find_package(${provider} REQUIRED CONFIG)
  get_filename_component(provider_prefix "${${provider}_DIR}/../../.." ABSOLUTE)
  if(NOT ${alias}_INCLUDE_DIR)
    set(${alias}_INCLUDE_DIR "${provider_prefix}/include")
  endif()
  if(alias STREQUAL "BOUNDARY_DEPARTURE_CHECKER")
    unset(BOUNDARY_DEPARTURE_CHECKER_LIBRARY CACHE)
  endif()
  if(NOT alias STREQUAL "UNIVERSE_UTILS" AND NOT ${alias}_LIBRARY)
    unset(${alias}_LIBRARY CACHE)
    set(provider_library_name "${provider}")
    if(provider STREQUAL "autoware_vehicle_info_utils")
      set(provider_library_name vehicle_info_utils)
    elseif(provider STREQUAL "autoware_deprecated_boundary_departure_checker")
      set(provider_library_name boundary_departure_checker)
    endif()
    find_library(${alias}_LIBRARY NAMES ${provider_library_name} HINTS "${provider_prefix}/lib" REQUIRED)
  endif()
endforeach()
foreach(path IN ITEMS MOTION_UTILS_INCLUDE_DIR MOTION_UTILS_LIBRARY INTERPOLATION_INCLUDE_DIR INTERPOLATION_LIBRARY UNIVERSE_UTILS_INCLUDE_DIR VEHICLE_INFO_UTILS_INCLUDE_DIR VEHICLE_INFO_UTILS_LIBRARY ROUTE_HANDLER_INCLUDE_DIR ROUTE_HANDLER_LIBRARY PLANNING_FACTOR_INTERFACE_INCLUDE_DIR PLANNING_FACTOR_INTERFACE_LIBRARY OBJECTS_OF_INTEREST_MARKER_INTERFACE_INCLUDE_DIR OBJECTS_OF_INTEREST_MARKER_INTERFACE_LIBRARY RTC_INTERFACE_INCLUDE_DIR RTC_INTERFACE_LIBRARY LANELET2_UTILS_INCLUDE_DIR LANELET2_UTILS_LIBRARY BOUNDARY_DEPARTURE_CHECKER_INCLUDE_DIR BOUNDARY_DEPARTURE_CHECKER_LIBRARY FREESPACE_ALGORITHMS_INCLUDE_DIR FREESPACE_ALGORITHMS_LIBRARY OBJECT_RECOGNITION_UTILS_INCLUDE_DIR OBJECT_RECOGNITION_UTILS_LIBRARY)
  if(NOT EXISTS "${${path}}")
    message(FATAL_ERROR "Set ${path} to its staged ROS1 provider")
  endif()
endforeach()

catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS autoware_adapi_v1_msgs autoware_deprecated_boundary_departure_checker autoware_internal_debug_msgs autoware_internal_planning_msgs autoware_perception_msgs autoware_planning_msgs autoware_traffic_light_utils autoware_vehicle_msgs autoware_motion_utils autoware_vehicle_info_utils autoware_route_handler autoware_lanelet2_utils autoware_lanelet2_extension autoware_freespace_planning_algorithms autoware_object_recognition_utils autoware_planning_factor_interface autoware_objects_of_interest_marker_interface autoware_rtc_interface geometry_msgs nav_msgs roscpp tf2 tf2_geometry_msgs tier4_planning_msgs unique_identifier_msgs autoware_utils visualization_msgs)
include_directories(include ${catkin_INCLUDE_DIRS} ${AW149_LANELET2_EXTENSION_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS} ${MOTION_UTILS_INCLUDE_DIR} ${INTERPOLATION_INCLUDE_DIR} ${UNIVERSE_UTILS_INCLUDE_DIR} ${VEHICLE_INFO_UTILS_INCLUDE_DIR} ${ROUTE_HANDLER_INCLUDE_DIR} ${PLANNING_FACTOR_INTERFACE_INCLUDE_DIR} ${OBJECTS_OF_INTEREST_MARKER_INTERFACE_INCLUDE_DIR} ${RTC_INTERFACE_INCLUDE_DIR} ${LANELET2_UTILS_INCLUDE_DIR} ${BOUNDARY_DEPARTURE_CHECKER_INCLUDE_DIR} ${FREESPACE_ALGORITHMS_INCLUDE_DIR} ${OBJECT_RECOGNITION_UTILS_INCLUDE_DIR})
add_library(${PROJECT_NAME} SHARED
  src/interface/data_readiness.cpp
  src/interface/bpp_input_core.cpp
  src/interface/scene_module_interface.cpp
  src/interface/scene_module_manager_interface.cpp
  src/interface/scene_module_visitor.cpp
  src/utils/route_utils.cpp
  src/utils/start_route_dependencies.cpp
  src/utils/start_geometry_dependencies.cpp
  src/utils/root_reference_utils.cpp
  src/utils/goal_geometry_dependencies.cpp
  src/planner_data.cpp
  src/utils/turn_lane_helpers.cpp
  src/turn_signal_decider.cpp
  src/turn_signal_resolver.cpp
  src/utils/traffic_light_utils.cpp
  src/utils/object_distance.cpp
  src/utils/parking_departure/velocity_utils.cpp
  src/utils/parking_departure/utils.cpp
  src/utils/parking_departure/geometric_parallel_parking_core.cpp
  src/utils/parking_departure/geometric_parallel_parking.cpp
  src/utils/parking_departure/geometric_dependencies.cpp
  src/utils/occupancy_grid_based_collision_detector/occupancy_grid_based_collision_detector.cpp
  src/utils/drivable_area_expansion/static_drivable_area.cpp
  src/utils/drivable_area_expansion/map_utils.cpp
  src/utils/drivable_area_expansion/footprints.cpp
  src/utils/drivable_area_expansion/dynamic_area_core.cpp
  src/utils/drivable_area_expansion/drivable_area_expansion.cpp
  src/utils/path_safety_checker/objects_filtering_core.cpp
  src/utils/path_safety_checker/objects_filtering.cpp
  src/utils/path_safety_checker/safety_core.cpp
  src/utils/path_safety_checker/safety_check.cpp
  src/marker_utils/utils.cpp
  src/marker_utils/start_markers.cpp
  src/utils/utils_ros1_compat.cpp
  src/utils/path_utils.cpp
  src/utils/path_shifter/path_shifter.cpp)
target_link_libraries(${PROJECT_NAME} ${MOTION_UTILS_LIBRARY} ${INTERPOLATION_LIBRARY} ${VEHICLE_INFO_UTILS_LIBRARY} ${ROUTE_HANDLER_LIBRARY} ${PLANNING_FACTOR_INTERFACE_LIBRARY} ${OBJECTS_OF_INTEREST_MARKER_INTERFACE_LIBRARY} ${RTC_INTERFACE_LIBRARY} ${LANELET2_UTILS_LIBRARY} ${BOUNDARY_DEPARTURE_CHECKER_LIBRARY} ${FREESPACE_ALGORITHMS_LIBRARY} ${OBJECT_RECOGNITION_UTILS_LIBRARY} ${LANELET2_CORE_LIBRARY} ${catkin_LIBRARIES})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_avoidance_object_ownership test/test_avoidance_object_ownership.cpp)
  if(TARGET test_avoidance_object_ownership)
    target_compile_features(test_avoidance_object_ownership PRIVATE cxx_std_17)
    target_link_libraries(test_avoidance_object_ownership gtest_main ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_path_shifter test/test_path_shifter.cpp)
  if(TARGET test_path_shifter)
    # Retained fixture is part of this export, not the upstream checkout.
    find_package(autoware_test_utils REQUIRED CONFIG)
    get_filename_component(AW149_TEST_UTILS_SHARE "${autoware_test_utils_DIR}/.." ABSOLUTE)
    target_compile_definitions(test_path_shifter PRIVATE AW149_ROUTE_FIXTURE_PATH="${AW149_TEST_UTILS_SHARE}/test_map/overlap_map.osm")
    target_link_libraries(test_path_shifter ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_geometric_parallel_parking_core test/test_geometric_parallel_parking_core.cpp)
  if(TARGET test_geometric_parallel_parking_core)
    target_link_libraries(test_geometric_parallel_parking_core ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_marker_colors test/test_marker_colors.cpp)
  if(TARGET test_marker_colors)
    target_link_libraries(test_marker_colors gtest_main ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_footprints test/test_footprints.cpp)
  if(TARGET test_footprints)
    target_link_libraries(test_footprints gtest_main ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_occupancy_grid_based_collision_detector test/test_occupancy_grid_based_collision_detector.cpp)
  if(TARGET test_occupancy_grid_based_collision_detector)
    target_link_libraries(test_occupancy_grid_based_collision_detector gtest_main ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_path_utils test/test_path_utils.cpp)
  if(TARGET test_path_utils)
    target_link_libraries(test_path_utils gtest_main ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_path_safety_parameter_surface test/test_path_safety_parameter_surface.cpp)
  if(TARGET test_path_safety_parameter_surface)
    target_link_libraries(test_path_safety_parameter_surface gtest_main ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_objects_filtering test/test_objects_filtering.cpp)
  if(TARGET test_objects_filtering)
    target_link_libraries(test_objects_filtering gtest_main ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_safety_check test/test_safety_check.cpp)
  if(TARGET test_safety_check)
    target_link_libraries(test_safety_check gtest_main ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_parking_departure_utils test/test_parking_departure_utils.cpp)
  if(TARGET test_parking_departure_utils)
    target_link_libraries(test_parking_departure_utils gtest_main ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_drivable_area_expansion test/test_drivable_area_expansion.cpp)
  if(TARGET test_drivable_area_expansion)
    target_link_libraries(test_drivable_area_expansion gtest_main ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_static_drivable_area test/test_static_drivable_area.cpp)
  if(TARGET test_static_drivable_area)
    target_link_libraries(test_static_drivable_area gtest_main ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_data_manager test/test_data_manager.cpp)
  if(TARGET test_data_manager)
    target_link_libraries(test_data_manager gtest_main ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_scene_module_visitor test/test_scene_module_visitor.cpp)
  if(TARGET test_scene_module_visitor)
    target_link_libraries(test_scene_module_visitor gtest_main ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_traffic_light_utils test/test_traffic_light_utils.cpp)
  if(TARGET test_traffic_light_utils)
    target_link_libraries(test_traffic_light_utils gtest_main ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_turn_signal test/test_turn_signal.cpp)
  if(TARGET test_turn_signal)
    target_link_libraries(test_turn_signal gtest_main ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_goal_refinement_core test/test_goal_refinement_core.cpp)
  if(TARGET test_goal_refinement_core)
    target_link_libraries(test_goal_refinement_core gtest_main ${PROJECT_NAME} ${catkin_LIBRARIES})
    target_compile_features(test_goal_refinement_core PRIVATE cxx_std_17)
  endif()
  catkin_add_gtest(test_goal_path_refinement_core test/test_goal_path_refinement_core.cpp)
  if(TARGET test_goal_path_refinement_core)
    target_link_libraries(test_goal_path_refinement_core gtest_main ${PROJECT_NAME} ${catkin_LIBRARIES})
    target_compile_features(test_goal_path_refinement_core PRIVATE cxx_std_17)
  endif()
endif()
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  foreach(probe IN ITEMS geometric_provider_pull_out_probe geometric_provider_pull_out_boundary_probe)
    add_executable(${probe} test/full_geometric/${probe}.cpp)
    target_compile_features(${probe} PRIVATE cxx_std_17)
    target_link_libraries(${probe} ${PROJECT_NAME} ${catkin_LIBRARIES} ${LANELET2_CORE_LIBRARY})
    add_test(NAME ${probe} COMMAND $<TARGET_FILE:${probe}>)
  endforeach()
endif()
install(DIRECTORY include/ DESTINATION include)
install(FILES package.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
