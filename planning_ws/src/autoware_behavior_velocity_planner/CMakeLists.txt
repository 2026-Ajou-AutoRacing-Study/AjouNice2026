cmake_minimum_required(VERSION 3.0.2)
project(autoware_behavior_velocity_planner)

find_package(catkin REQUIRED COMPONENTS
  autoware_internal_planning_msgs
  autoware_map_msgs
  autoware_motion_utils
  autoware_perception_msgs
  autoware_planning_msgs
  autoware_route_handler
  autoware_vehicle_info_utils
  autoware_velocity_smoother
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  pluginlib
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf2_eigen
  tf2_ros
)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES autoware_behavior_velocity_planner_manager autoware_behavior_velocity_planner_node autoware_behavior_velocity_planner_full_source
  CATKIN_DEPENDS autoware_internal_planning_msgs
    autoware_map_msgs autoware_motion_utils autoware_perception_msgs autoware_planning_msgs autoware_route_handler
    autoware_vehicle_info_utils
    autoware_velocity_smoother
    diagnostic_msgs geometry_msgs nav_msgs pluginlib pcl_conversions pcl_ros roscpp sensor_msgs tf2_ros
  DEPENDS PCL
)

include("${CMAKE_CURRENT_LIST_DIR}/../cmake/AW149Providers.cmake")
aw149_provider(LANELET2_UTILS autoware_lanelet2_utils autoware_lanelet2_utils)
aw149_provider(BVPC_COMMON autoware_behavior_velocity_planner_common autoware_behavior_velocity_planner_common)
aw149_provider(AUTOWARE_UTILS_PCL autoware_utils_pcl NONE)
aw149_provider(AUTOWARE_TRAJECTORY autoware_trajectory autoware_trajectory)
set(BVPC_COMMON_INCLUDE_DIR "" CACHE PATH
  "Installed behavior-velocity planner common include directory for the manager-only closure")
if(NOT EXISTS "${BVPC_COMMON_INCLUDE_DIR}/autoware/behavior_velocity_planner_common/plugin_interface.hpp")
  message(FATAL_ERROR "Set BVPC_COMMON_INCLUDE_DIR to the staged common install/include directory")
endif()
set(BVPC_COMMON_LIBRARY "" CACHE FILEPATH
  "Installed behavior-velocity planner common library for the ROS1 node runtime closure")
if(NOT EXISTS "${BVPC_COMMON_LIBRARY}")
  message(FATAL_ERROR "Set BVPC_COMMON_LIBRARY to the staged common install/lib library")
endif()
set(AUTOWARE_UTILS_PCL_INCLUDE_DIR "" CACHE PATH
  "Installed header-only autoware_utils_pcl include directory for the ROS1 node runtime closure")
if(NOT EXISTS "${AUTOWARE_UTILS_PCL_INCLUDE_DIR}/autoware_utils_pcl/transforms.hpp")
  message(FATAL_ERROR "Set AUTOWARE_UTILS_PCL_INCLUDE_DIR to the staged autoware_utils_pcl install/include directory")
endif()
set(AUTOWARE_TRAJECTORY_INCLUDE_DIR "" CACHE PATH
  "Installed trajectory include directory required by the experimental BVP ABI")
set(AUTOWARE_TRAJECTORY_LIBRARY "" CACHE FILEPATH
  "Installed trajectory library required by the experimental BVP ABI")
if(NOT EXISTS "${AUTOWARE_TRAJECTORY_INCLUDE_DIR}/autoware/trajectory/path_point_with_lane_id.hpp" OR
   NOT EXISTS "${AUTOWARE_TRAJECTORY_LIBRARY}")
  message(FATAL_ERROR "Set AUTOWARE_TRAJECTORY_INCLUDE_DIR and AUTOWARE_TRAJECTORY_LIBRARY to the staged trajectory provider")
endif()
set(LANELET2_UTILS_INCLUDE_DIR "" CACHE PATH
  "Installed lanelet2-utils include directory for map-provider integration tests")
set(LANELET2_UTILS_LIBRARY "" CACHE FILEPATH
  "Installed lanelet2-utils library for map-provider integration tests")

add_library(autoware_behavior_velocity_planner_manager SHARED
  src/planner_manager.cpp
  src/experimental/planner_manager.cpp
)
target_include_directories(autoware_behavior_velocity_planner_manager PUBLIC
  include ${BVPC_COMMON_INCLUDE_DIR} ${AUTOWARE_TRAJECTORY_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
target_link_libraries(autoware_behavior_velocity_planner_manager PUBLIC
  "${AUTOWARE_TRAJECTORY_LIBRARY}" ${catkin_LIBRARIES})
target_compile_features(autoware_behavior_velocity_planner_manager PUBLIC cxx_std_17)

add_library(autoware_behavior_velocity_planner_node SHARED
  src/node.cpp
  src/experimental/node.cpp
  src/smoother_initialization.cpp
)
target_include_directories(autoware_behavior_velocity_planner_node PUBLIC
  include ${BVPC_COMMON_INCLUDE_DIR} ${AUTOWARE_UTILS_PCL_INCLUDE_DIR} ${AUTOWARE_TRAJECTORY_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
target_link_libraries(autoware_behavior_velocity_planner_node PUBLIC
  autoware_behavior_velocity_planner_manager "${BVPC_COMMON_LIBRARY}" "${AUTOWARE_TRAJECTORY_LIBRARY}" ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_compile_features(autoware_behavior_velocity_planner_node PUBLIC cxx_std_17)

add_executable(behavior_velocity_planner_node src/main.cpp)
target_link_libraries(behavior_velocity_planner_node
  autoware_behavior_velocity_planner_node ${catkin_LIBRARIES})

add_executable(behavior_velocity_planner_experimental_node src/experimental_main.cpp)
target_link_libraries(behavior_velocity_planner_experimental_node
  autoware_behavior_velocity_planner_node ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_behavior_velocity_planner_manager test/test_planner_manager.cpp)
  if(TARGET test_behavior_velocity_planner_manager)
    target_compile_definitions(test_behavior_velocity_planner_manager PRIVATE
      AW149_BVP_NO_STOPPING_AREA_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/../autoware_behavior_velocity_no_stopping_area_module/config/ros1/no_stopping_area.yaml")
    target_link_libraries(test_behavior_velocity_planner_manager
      gtest_main autoware_behavior_velocity_planner_manager ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(
    test_behavior_velocity_planner_experimental_manager
    test/test_experimental_planner_manager.cpp)
  if(TARGET test_behavior_velocity_planner_experimental_manager)
    target_compile_definitions(test_behavior_velocity_planner_experimental_manager PRIVATE
      AW149_BVP_NO_STOPPING_AREA_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/../autoware_behavior_velocity_no_stopping_area_module/config/ros1/no_stopping_area.yaml"
      AW149_BVP_COMMON_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/ros1/behavior_velocity_planner.yaml"
      AW149_BVP_INTERSECTION_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/../autoware_behavior_velocity_intersection_module/config/ros1/intersection.yaml"
      AW149_BVP_CROSSWALK_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/../autoware_behavior_velocity_crosswalk_module/config/ros1/crosswalk.yaml"
      AW149_BVP_DETECTION_AREA_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/../autoware_behavior_velocity_detection_area_module/config/ros1/detection_area.yaml"
      AW149_BVP_TRAFFIC_LIGHT_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/../autoware_behavior_velocity_traffic_light_module/config/ros1/traffic_light.yaml"
      AW149_BVP_BLIND_SPOT_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/../autoware_behavior_velocity_blind_spot_module/config/ros1/blind_spot.yaml"
      AW149_BVP_ROUNDABOUT_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/../autoware_behavior_velocity_roundabout_module/config/ros1/roundabout.yaml"
      AW149_BVP_WALKWAY_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/../autoware_behavior_velocity_walkway_module/config/ros1/walkway.yaml"
      AW149_BVP_OCCLUSION_SPOT_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/../autoware_behavior_velocity_occlusion_spot_module/config/ros1/occlusion_spot.yaml")
    target_link_libraries(test_behavior_velocity_planner_experimental_manager
      autoware_behavior_velocity_planner_manager ${catkin_LIBRARIES})
  endif()
endif()

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_behavior_velocity_planner_node test/test_node_transport.cpp)
  if(TARGET test_behavior_velocity_planner_node)
    target_compile_definitions(test_behavior_velocity_planner_node PRIVATE
      AW149_BVP_TRANSPORT_TEST_MAP="${CMAKE_CURRENT_SOURCE_DIR}/test_map/lanelet2_map.osm"
      AW149_BVP_SMOOTHER_COMMON_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/../autoware_velocity_smoother/config/ros1/default_velocity_smoother.yaml"
      AW149_BVP_SMOOTHER_ANALYTICAL_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/../autoware_velocity_smoother/config/ros1/Analytical.yaml"
      AW149_BVP_DETECTION_AREA_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/../autoware_behavior_velocity_detection_area_module/config/ros1/detection_area.yaml"
      AW149_BVP_NO_STOPPING_AREA_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/../autoware_behavior_velocity_no_stopping_area_module/config/ros1/no_stopping_area.yaml")
    if(NOT EXISTS "${LANELET2_UTILS_INCLUDE_DIR}/autoware/lanelet2_utils/conversion.hpp" OR
       NOT EXISTS "${LANELET2_UTILS_LIBRARY}")
      message(FATAL_ERROR "Set LANELET2_UTILS_INCLUDE_DIR and LANELET2_UTILS_LIBRARY for planner map tests")
    endif()
    target_include_directories(test_behavior_velocity_planner_node PRIVATE ${LANELET2_UTILS_INCLUDE_DIR})
    target_link_libraries(test_behavior_velocity_planner_node
      autoware_behavior_velocity_planner_node "${LANELET2_UTILS_LIBRARY}" ${catkin_LIBRARIES})
  endif()

  # CTest must exercise the same explicit candidate closure as the foreground
  # BVP runtime gate.  Without this, pluginlib silently sees only Noetic and
  # reports every candidate scene module as unavailable, which turns policy
  # tests into ambient-workspace probes.
  # Resolve only the exported workspace. All scene plugins must be installed
  # before the integration tests run; missing plugins remain test failures.
  set(AW149_BVP_TEST_PREFIXES "${CATKIN_DEVEL_PREFIX}" "${CMAKE_INSTALL_PREFIX}")
  set(AW149_BVP_TEST_ROS_PATHS "${CMAKE_CURRENT_SOURCE_DIR}")
  set(AW149_BVP_TEST_LIBRARIES "")
  foreach(prefix IN LISTS AW149_BVP_TEST_PREFIXES)
    # Output prefixes need not exist during a first clean configure.
    # Runtime tests still require all installed scene providers.
    list(APPEND AW149_BVP_TEST_LIBRARIES "${prefix}/lib")
    list(APPEND AW149_BVP_TEST_ROS_PATHS "${prefix}/share")
  endforeach()
  list(APPEND AW149_BVP_TEST_ROS_PATHS "/opt/ros/noetic/share")
  list(APPEND AW149_BVP_TEST_LIBRARIES "/opt/ros/noetic/lib" "/opt/ros/noetic/lib/x86_64-linux-gnu")
  string(JOIN ":" AW149_BVP_TEST_PREFIX_PATH ${AW149_BVP_TEST_PREFIXES})
  string(JOIN ":" AW149_BVP_TEST_ROS_PATH ${AW149_BVP_TEST_ROS_PATHS})
  string(JOIN ":" AW149_BVP_TEST_LIBRARY_PATH ${AW149_BVP_TEST_LIBRARIES})
  foreach(test_name IN ITEMS
      _ctest_${PROJECT_NAME}_gtest_test_behavior_velocity_planner_manager
      _ctest_${PROJECT_NAME}_gtest_test_behavior_velocity_planner_experimental_manager
      _ctest_${PROJECT_NAME}_gtest_test_behavior_velocity_planner_node)
    set_tests_properties(${test_name} PROPERTIES ENVIRONMENT
      "CMAKE_PREFIX_PATH=${AW149_BVP_TEST_PREFIX_PATH}:$ENV{CMAKE_PREFIX_PATH};ROS_PACKAGE_PATH=${AW149_BVP_TEST_ROS_PATH}:$ENV{ROS_PACKAGE_PATH};LD_LIBRARY_PATH=${AW149_BVP_TEST_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH}")
  endforeach()
endif()

install(TARGETS autoware_behavior_velocity_planner_manager autoware_behavior_velocity_planner_node
  behavior_velocity_planner_node behavior_velocity_planner_experimental_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES include/autoware/behavior_velocity_planner/planner_manager.hpp
  include/autoware/behavior_velocity_planner/node.hpp
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/autoware/behavior_velocity_planner
)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES
  include/autoware/behavior_velocity_planner/experimental/planner_manager.hpp
  include/autoware/behavior_velocity_planner/experimental/node.hpp
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/autoware/behavior_velocity_planner/experimental
)

find_package(autoware_utils_rclcpp REQUIRED CONFIG)
find_package(autoware_utils_debug REQUIRED CONFIG)
find_package(autoware_utils_logging REQUIRED CONFIG)
find_package(autoware_utils_math REQUIRED CONFIG)
find_package(autoware_planning_test_manager REQUIRED CONFIG)
find_package(yaml-cpp REQUIRED)
add_library(autoware_behavior_velocity_planner_full_source SHARED src/full_node.cpp src/experimental/full_node.cpp src/test_utils.cpp)
target_include_directories(autoware_behavior_velocity_planner_full_source PUBLIC ${autoware_utils_math_INCLUDE_DIRS} ${autoware_planning_test_manager_INCLUDE_DIRS})
target_include_directories(autoware_behavior_velocity_planner_full_source BEFORE PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(autoware_behavior_velocity_planner_full_source PUBLIC cxx_std_17)
target_link_libraries(autoware_behavior_velocity_planner_full_source autoware_behavior_velocity_planner_node
  ${autoware_utils_rclcpp_LIBRARIES} ${autoware_utils_debug_LIBRARIES} ${autoware_utils_logging_LIBRARIES} ${autoware_planning_test_manager_LIBRARIES} yaml-cpp)
set_target_properties(autoware_behavior_velocity_planner_full_source PROPERTIES LINK_FLAGS "-Wl,--no-undefined")
add_executable(behavior_velocity_planner_full_node src/full_main.cpp)
target_link_libraries(behavior_velocity_planner_full_node autoware_behavior_velocity_planner_full_source)
add_executable(behavior_velocity_planner_full_experimental_node src/full_experimental_main.cpp)
target_link_libraries(behavior_velocity_planner_full_experimental_node autoware_behavior_velocity_planner_full_source)
install(TARGETS autoware_behavior_velocity_planner_full_source behavior_velocity_planner_full_node behavior_velocity_planner_full_experimental_node
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES include/autoware/behavior_velocity_planner/full_node.hpp
  include/autoware/behavior_velocity_planner/full_native_support.hpp
  include/autoware/behavior_velocity_planner/test_utils.hpp
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/autoware/behavior_velocity_planner)
install(FILES include/autoware/behavior_velocity_planner/experimental/full_node.hpp
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/autoware/behavior_velocity_planner/experimental)
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_full_bvp_original_node test/test_full_original_node_interface.cpp)
  if(TARGET test_full_bvp_original_node)
    target_link_libraries(test_full_bvp_original_node autoware_behavior_velocity_planner_full_source)
  endif()
endif()
