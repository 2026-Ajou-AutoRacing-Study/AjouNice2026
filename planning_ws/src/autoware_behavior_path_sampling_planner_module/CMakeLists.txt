cmake_minimum_required(VERSION 3.0.2)
project(autoware_behavior_path_sampling_planner_module)

find_package(catkin REQUIRED COMPONENTS pluginlib roscpp geometry_msgs visualization_msgs nav_msgs tf2 tf2_geometry_msgs)
find_package(Eigen3 REQUIRED)

# Every non-ROS dependency is injected from a sealed ROS1 provider receipt.
# This prevents accidentally compiling against the ROS2 source tree or ambient overlays.
include("${CMAKE_CURRENT_LIST_DIR}/../cmake/AW149Providers.cmake")
aw149_provider(BEZIER autoware_bezier_sampler autoware_bezier_sampler)
aw149_provider(BOUNDARY_DEPARTURE_CHECKER_UTILS autoware_deprecated_boundary_departure_checker boundary_departure_checker_utils)
aw149_provider(BPP_COMMON autoware_behavior_path_planner_common autoware_behavior_path_planner_common)
aw149_provider(FRENET autoware_frenet_planner autoware_frenet_planner)
aw149_provider(GEOMETRY_UTILS autoware_utils_geometry autoware_utils_geometry)
aw149_provider(INTERPOLATION autoware_interpolation autoware_interpolation)
aw149_provider(LANELET2_CORE lanelet2_core lanelet2_core)
aw149_provider(LANELET2_UTILS autoware_lanelet2_utils autoware_lanelet2_utils)
aw149_provider(MOTION_UTILS autoware_motion_utils autoware_motion_utils)
aw149_provider(OBJECTS_OF_INTEREST autoware_objects_of_interest_marker_interface autoware_objects_of_interest_marker_interface)
aw149_provider(OBJECT_RECOGNITION_UTILS autoware_object_recognition_utils autoware_object_recognition_utils)
aw149_provider(PLANNING_FACTOR autoware_planning_factor_interface autoware_planning_factor_interface)
aw149_provider(PLANNING_MSGS autoware_planning_msgs NONE)
aw149_provider(ROUTE_HANDLER autoware_route_handler autoware_route_handler)
aw149_provider(RTC autoware_rtc_interface autoware_rtc_interface)
aw149_provider(RTC_INTERFACE autoware_rtc_interface autoware_rtc_interface)
aw149_provider(SAMPLER_COMMON autoware_sampler_common autoware_sampler_common)
aw149_provider(TIER4_PLANNING_MSGS tier4_planning_msgs NONE)
aw149_provider(TRAFFIC_LIGHT_UTILS autoware_traffic_light_utils autoware_traffic_light_utils)
aw149_provider(UNIVERSE_UTILS autoware_universe_utils NONE)
aw149_provider(VEHICLE_INFO_UTILS autoware_vehicle_info_utils vehicle_info_utils)
set(BPP_COMMON_INCLUDE_DIR "" CACHE PATH "staged BPP common include")
set(BPP_COMMON_LIBRARY "" CACHE FILEPATH "staged BPP common library")
set(BEZIER_INCLUDE_DIR "" CACHE PATH "staged Bezier sampler include")
set(BEZIER_LIBRARY "" CACHE FILEPATH "staged Bezier sampler library")
set(FRENET_INCLUDE_DIR "" CACHE PATH "staged Frenet planner include")
set(FRENET_LIBRARY "" CACHE FILEPATH "staged Frenet planner library")
set(SAMPLER_COMMON_INCLUDE_DIR "" CACHE PATH "staged sampler common include")
set(SAMPLER_COMMON_LIBRARY "" CACHE FILEPATH "staged sampler common library")
set(LANELET2_UTILS_INCLUDE_DIR "" CACHE PATH "staged lanelet2-utils include")
set(LANELET2_UTILS_LIBRARY "" CACHE FILEPATH "staged lanelet2-utils library")
set(MOTION_UTILS_INCLUDE_DIR "" CACHE PATH "staged motion-utils include")
set(MOTION_UTILS_LIBRARY "" CACHE FILEPATH "staged motion-utils library")
set(ROUTE_HANDLER_INCLUDE_DIR "" CACHE PATH "staged route-handler include")
set(ROUTE_HANDLER_LIBRARY "" CACHE FILEPATH "staged route-handler library")
set(UNIVERSE_UTILS_INCLUDE_DIR "" CACHE PATH "staged universe-utils include")
set(VEHICLE_INFO_UTILS_INCLUDE_DIR "" CACHE PATH "staged vehicle-info include")
set(VEHICLE_INFO_UTILS_LIBRARY "" CACHE FILEPATH "staged vehicle-info library")
set(PLANNING_MSGS_INCLUDE_DIR "" CACHE PATH "staged planning/internal message include")
set(TIER4_PLANNING_MSGS_INCLUDE_DIR "" CACHE PATH "staged tier4 planning messages include")
set(GEOMETRY_UTILS_LIBRARY "" CACHE FILEPATH "staged autoware-utils geometry library")
set(INTERPOLATION_LIBRARY "" CACHE FILEPATH "staged interpolation library")
set(RTC_LIBRARY "" CACHE FILEPATH "staged RTC interface library")
set(LANELET2_CORE_LIBRARY "" CACHE FILEPATH "sealed lanelet2 core library")
set(OBJECTS_OF_INTEREST_INCLUDE_DIR "" CACHE PATH "staged objects-of-interest interface include")
set(PLANNING_FACTOR_INCLUDE_DIR "" CACHE PATH "staged planning-factor interface include")
set(RTC_INTERFACE_INCLUDE_DIR "" CACHE PATH "staged RTC interface include")
set(OBJECTS_OF_INTEREST_LIBRARY "" CACHE FILEPATH "staged objects-of-interest interface library")
set(OBJECT_RECOGNITION_UTILS_LIBRARY "" CACHE FILEPATH "staged object-recognition utility library")
set(TRAFFIC_LIGHT_UTILS_LIBRARY "" CACHE FILEPATH "staged traffic-light utility library")
set(BOUNDARY_DEPARTURE_CHECKER_UTILS_LIBRARY "" CACHE FILEPATH "staged boundary-departure utility library")

foreach(provider IN ITEMS
  BPP_COMMON_INCLUDE_DIR BPP_COMMON_LIBRARY BEZIER_INCLUDE_DIR BEZIER_LIBRARY
  FRENET_INCLUDE_DIR FRENET_LIBRARY SAMPLER_COMMON_INCLUDE_DIR SAMPLER_COMMON_LIBRARY
  LANELET2_UTILS_INCLUDE_DIR LANELET2_UTILS_LIBRARY MOTION_UTILS_INCLUDE_DIR MOTION_UTILS_LIBRARY
  ROUTE_HANDLER_INCLUDE_DIR ROUTE_HANDLER_LIBRARY UNIVERSE_UTILS_INCLUDE_DIR
  VEHICLE_INFO_UTILS_INCLUDE_DIR VEHICLE_INFO_UTILS_LIBRARY PLANNING_MSGS_INCLUDE_DIR
  TIER4_PLANNING_MSGS_INCLUDE_DIR OBJECTS_OF_INTEREST_INCLUDE_DIR PLANNING_FACTOR_INCLUDE_DIR
  RTC_INTERFACE_INCLUDE_DIR GEOMETRY_UTILS_LIBRARY INTERPOLATION_LIBRARY RTC_LIBRARY
  LANELET2_CORE_LIBRARY OBJECTS_OF_INTEREST_LIBRARY OBJECT_RECOGNITION_UTILS_LIBRARY
  TRAFFIC_LIGHT_UTILS_LIBRARY BOUNDARY_DEPARTURE_CHECKER_UTILS_LIBRARY)
  if(NOT EXISTS "${${provider}}")
    message(FATAL_ERROR "Set ${provider} to its sealed staged ROS1 provider; ambient discovery is forbidden")
  endif()
endforeach()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS pluginlib roscpp geometry_msgs visualization_msgs nav_msgs tf2 tf2_geometry_msgs
)

add_library(${PROJECT_NAME} SHARED src/manager.cpp src/sampling_planner_module.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
# Candidate DSOs may not carry unresolved direct references into the plugin loader.
target_link_options(${PROJECT_NAME} PRIVATE -Wl,-z,defs)
target_include_directories(${PROJECT_NAME}
  PUBLIC include
  SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
    ${BPP_COMMON_INCLUDE_DIR} ${BEZIER_INCLUDE_DIR} ${FRENET_INCLUDE_DIR} ${SAMPLER_COMMON_INCLUDE_DIR}
    ${LANELET2_UTILS_INCLUDE_DIR} ${MOTION_UTILS_INCLUDE_DIR} ${ROUTE_HANDLER_INCLUDE_DIR}
    ${UNIVERSE_UTILS_INCLUDE_DIR} ${VEHICLE_INFO_UTILS_INCLUDE_DIR} ${PLANNING_MSGS_INCLUDE_DIR}
    ${TIER4_PLANNING_MSGS_INCLUDE_DIR} ${OBJECTS_OF_INTEREST_INCLUDE_DIR}
    ${PLANNING_FACTOR_INCLUDE_DIR} ${RTC_INTERFACE_INCLUDE_DIR}
)
target_link_libraries(${PROJECT_NAME}
  ${BPP_COMMON_LIBRARY} ${BEZIER_LIBRARY} ${FRENET_LIBRARY} ${SAMPLER_COMMON_LIBRARY}
  ${LANELET2_UTILS_LIBRARY} ${MOTION_UTILS_LIBRARY} ${ROUTE_HANDLER_LIBRARY}
  ${VEHICLE_INFO_UTILS_LIBRARY} ${GEOMETRY_UTILS_LIBRARY} ${INTERPOLATION_LIBRARY}
  ${RTC_LIBRARY} ${LANELET2_CORE_LIBRARY} ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_sampling_planner_plugin test/test_sampling_planner_plugin.cpp)
  if(TARGET test_sampling_planner_plugin)
    # BPP Common exposes these source-owned helpers as DSO dependencies.  Link
    # the test closure directly so the plugin gate cannot pass through an
    # ambient loader path or fail only after class-loader construction.
    target_link_libraries(test_sampling_planner_plugin ${PROJECT_NAME} ${catkin_LIBRARIES} gtest_main
      ${OBJECTS_OF_INTEREST_LIBRARY} ${OBJECT_RECOGNITION_UTILS_LIBRARY}
      ${TRAFFIC_LIGHT_UTILS_LIBRARY} ${BOUNDARY_DEPARTURE_CHECKER_UTILS_LIBRARY})
  endif()
endif()

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(FILES package.xml plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

find_package(autoware_utils_rclcpp REQUIRED)
include_directories(BEFORE ${autoware_utils_rclcpp_INCLUDE_DIRS})
