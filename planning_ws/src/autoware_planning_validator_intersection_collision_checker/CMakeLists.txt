cmake_minimum_required(VERSION 3.0.2)
project(autoware_planning_validator_intersection_collision_checker)
find_package(autoware_traffic_light_utils REQUIRED)

find_package(catkin REQUIRED COMPONENTS pluginlib roscpp pcl_conversions rostest tf2_ros)
find_package(autoware_test_utils REQUIRED CONFIG)
find_package(PCL REQUIRED COMPONENTS common filters kdtree segmentation surface)
find_package(Eigen3 REQUIRED)
unset(AW149_LANELET2_CORE_LIBRARY CACHE)
find_library(AW149_LANELET2_CORE_LIBRARY NAMES lanelet2_core PATHS /opt/ros/noetic/lib/x86_64-linux-gnu NO_DEFAULT_PATH)
find_package(autoware_planning_validator REQUIRED CONFIG)
get_filename_component(AW149_VALIDATOR_PREFIX "${autoware_planning_validator_DIR}/../../.." ABSOLUTE)
find_package(autoware_route_handler REQUIRED CONFIG)
get_filename_component(AW149_ROUTE_HANDLER_PREFIX "${autoware_route_handler_DIR}/../../.." ABSOLUTE)
find_package(autoware_lanelet2_utils REQUIRED CONFIG)
get_filename_component(AW149_INTERFACE_PREFIX "${autoware_lanelet2_utils_DIR}/../../.." ABSOLUTE)
find_package(autoware_vehicle_info_utils REQUIRED CONFIG)
get_filename_component(AW149_VEHICLE_INFO_PREFIX "${autoware_vehicle_info_utils_DIR}/../../.." ABSOLUTE)
set(AW149_ICC_RUNTIME_BUILD "${CATKIN_DEVEL_PREFIX}/lib")
add_subdirectory(runtime)
if(NOT EXISTS "${AW149_VALIDATOR_PREFIX}/include/autoware/planning_validator/plugin_interface.hpp")
  message(FATAL_ERROR "Missing Planning Validator R002 ABI")
endif()
unset(AW149_ROUTE_HANDLER_LIBRARY CACHE)
find_library(AW149_ROUTE_HANDLER_LIBRARY NAMES autoware_route_handler PATHS "${AW149_ROUTE_HANDLER_PREFIX}/lib" NO_DEFAULT_PATH)
unset(AW149_LANELET2_UTILS_LIBRARY CACHE)
find_library(AW149_LANELET2_UTILS_LIBRARY NAMES autoware_lanelet2_utils PATHS "${AW149_INTERFACE_PREFIX}/lib" NO_DEFAULT_PATH)
unset(AW149_VALIDATOR_MANAGER_LIBRARY CACHE)
find_library(AW149_VALIDATOR_MANAGER_LIBRARY NAMES planning_validator_manager_ros1 PATHS "${AW149_VALIDATOR_PREFIX}/lib" NO_DEFAULT_PATH)
set(AW149_ICC_LANELET_SELECTION_LIBRARY lanelet_selection_core_ros1)
foreach(AW149_ICC_CORE
  ego_trajectory_core_ros1
  right_active_target_core_ros1
  left_active_target_core_ros1
  traffic_regulatory_priority_core_ros1
  active_turn_collision_core_ros1
  pointcloud_preprocess_core_ros1)
  string(TOUPPER "${AW149_ICC_CORE}" AW149_ICC_CORE_UPPER)
  set(AW149_${AW149_ICC_CORE_UPPER}_LIBRARY ${AW149_ICC_CORE})
  if(NOT AW149_${AW149_ICC_CORE_UPPER}_LIBRARY)
    message(FATAL_ERROR "Missing foreground-built ICC core: ${AW149_ICC_CORE}")
  endif()
endforeach()
if(NOT AW149_ROUTE_HANDLER_LIBRARY)
  message(FATAL_ERROR "Missing RouteHandler ABI library")
endif()
if(NOT AW149_ICC_LANELET_SELECTION_LIBRARY)
  message(FATAL_ERROR "Missing foreground-built ICC lanelet-selection core")
endif()
set(CMAKE_BUILD_RPATH "${AW149_ROUTE_HANDLER_PREFIX}/lib;${AW149_INTERFACE_PREFIX}/lib;${AW149_ICC_RUNTIME_BUILD}")

catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME}_full_source ${PROJECT_NAME} CATKIN_DEPENDS pluginlib roscpp)
include_directories(include support/include runtime/include "${AW149_VALIDATOR_PREFIX}/include" "${AW149_ROUTE_HANDLER_PREFIX}/include" "${AW149_INTERFACE_PREFIX}/include" "${AW149_INTERFACE_PREFIX}/include/autoware_lanelet2_extension" "${AW149_VEHICLE_INFO_PREFIX}/include" ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_library(${PROJECT_NAME} SHARED src/intersection_collision_checker_plugin.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${AW149_ICC_LANELET_SELECTION_LIBRARY}
  ${AW149_EGO_TRAJECTORY_CORE_ROS1_LIBRARY}
  ${AW149_RIGHT_ACTIVE_TARGET_CORE_ROS1_LIBRARY}
  ${AW149_LEFT_ACTIVE_TARGET_CORE_ROS1_LIBRARY}
  ${AW149_TRAFFIC_REGULATORY_PRIORITY_CORE_ROS1_LIBRARY}
  ${AW149_ACTIVE_TURN_COLLISION_CORE_ROS1_LIBRARY}
  ${AW149_POINTCLOUD_PREPROCESS_CORE_ROS1_LIBRARY}
  ${AW149_ROUTE_HANDLER_LIBRARY} ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(intersection_collision_checker_provider_oracle test/provider_oracle.cpp)
  if(TARGET intersection_collision_checker_provider_oracle)
    target_link_libraries(intersection_collision_checker_provider_oracle ${PROJECT_NAME} ${catkin_LIBRARIES})
    target_compile_features(intersection_collision_checker_provider_oracle PUBLIC cxx_std_17)
    set_tests_properties(
      _ctest_${PROJECT_NAME}_gtest_intersection_collision_checker_provider_oracle
      PROPERTIES ENVIRONMENT
      "LD_LIBRARY_PATH=${AW149_ROUTE_HANDLER_PREFIX}/lib:${AW149_INTERFACE_PREFIX}/lib:$ENV{LD_LIBRARY_PATH}")
  endif()
  catkin_add_gtest(intersection_collision_checker_pluginlib_oracle test/pluginlib_oracle.cpp)
  if(TARGET intersection_collision_checker_pluginlib_oracle)
    target_link_libraries(intersection_collision_checker_pluginlib_oracle ${catkin_LIBRARIES})
    target_compile_features(intersection_collision_checker_pluginlib_oracle PUBLIC cxx_std_17)
    set_tests_properties(
      _ctest_${PROJECT_NAME}_gtest_intersection_collision_checker_pluginlib_oracle
      PROPERTIES ENVIRONMENT
      "ROS_PACKAGE_PATH=${CMAKE_INSTALL_PREFIX}/share:${AW149_VALIDATOR_PREFIX}/share:$ENV{ROS_PACKAGE_PATH};CMAKE_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:${AW149_VALIDATOR_PREFIX}:$ENV{CMAKE_PREFIX_PATH};LD_LIBRARY_PATH=${CMAKE_INSTALL_PREFIX}/lib:${AW149_ICC_RUNTIME_BUILD}:${AW149_ROUTE_HANDLER_PREFIX}/lib:${AW149_INTERFACE_PREFIX}/lib:$ENV{LD_LIBRARY_PATH}")
  endif()
  # This oracle needs the complete ROS parameter tree. It is disabled as a
  # bare CTest and launched by rostest, which owns its master only while this
  # foreground fixture runs.
  catkin_add_gtest(intersection_collision_checker_provider_active_oracle test/provider_active_oracle.cpp)
  if(TARGET intersection_collision_checker_provider_active_oracle)
    target_link_libraries(intersection_collision_checker_provider_active_oracle ${PROJECT_NAME} ${AW149_VALIDATOR_MANAGER_LIBRARY} ${AW149_ROUTE_HANDLER_LIBRARY} ${AW149_LANELET2_UTILS_LIBRARY} ${AW149_LANELET2_CORE_LIBRARY} ${PCL_LIBRARIES} ${catkin_LIBRARIES})
    target_compile_features(intersection_collision_checker_provider_active_oracle PUBLIC cxx_std_17)
    target_compile_definitions(intersection_collision_checker_provider_active_oracle PRIVATE
      AW149_INTERSECTION_FIXTURE_PATH="${autoware_test_utils_DIR}/../test_map/intersection/lanelet2_map.osm")
    set_tests_properties(
      _ctest_${PROJECT_NAME}_gtest_intersection_collision_checker_provider_active_oracle
      PROPERTIES DISABLED TRUE ENVIRONMENT
      "LD_LIBRARY_PATH=${AW149_ICC_RUNTIME_BUILD}:${AW149_ROUTE_HANDLER_PREFIX}/lib:${AW149_INTERFACE_PREFIX}/lib:$ENV{LD_LIBRARY_PATH}")
  endif()
endif()

install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(DIRECTORY config/ros1 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(FILES package.xml plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
# The parent validator launch consumes the complete candidate-only ROS1 tree;
# install it with the provider instead of depending on a test-only fixture.
install(FILES config/intersection_collision_checker.ros1.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(DIRECTORY support/include/ runtime/include/ DESTINATION include)

# Full original implementation, separate ABI from retained providers.
find_package(autoware_utils REQUIRED)
find_package(autoware_utils_debug REQUIRED)
find_package(autoware_utils_rclcpp REQUIRED)
find_package(autoware_utils_logging REQUIRED)
find_package(autoware_planning_factor_interface REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters kdtree segmentation surface)
include_directories(BEFORE include)
include_directories(${PCL_INCLUDE_DIRS} ${autoware_planning_validator_INCLUDE_DIRS} ${autoware_utils_INCLUDE_DIRS})
add_library(${PROJECT_NAME}_full_source src/full_source/intersection_collision_checker.cpp src/full_source/utils.cpp)
target_compile_features(${PROJECT_NAME}_full_source PUBLIC cxx_std_17)
add_dependencies(${PROJECT_NAME}_full_source ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_full_source ${autoware_planning_validator_LIBRARIES}
  ${autoware_traffic_light_utils_LIBRARIES}
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${autoware_utils_LIBRARIES}
  ${autoware_utils_debug_LIBRARIES} ${autoware_utils_logging_LIBRARIES}
  ${autoware_planning_factor_interface_LIBRARIES} ${diagnostic_updater_LIBRARIES})
set_target_properties(${PROJECT_NAME}_full_source PROPERTIES LINK_FLAGS "-Wl,--no-undefined")
install(TARGETS ${PROJECT_NAME}_full_source
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES full_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(full_parameter_contract test/full_parameter_contract.cpp)
  if(TARGET full_parameter_contract)
    target_compile_features(full_parameter_contract PUBLIC cxx_std_17)
    target_link_libraries(full_parameter_contract ${catkin_LIBRARIES})
  endif()
endif()
