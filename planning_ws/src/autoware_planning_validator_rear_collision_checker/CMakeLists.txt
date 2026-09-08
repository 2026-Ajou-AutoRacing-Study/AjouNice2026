cmake_minimum_required(VERSION 3.0.2)
project(autoware_planning_validator_rear_collision_checker)
find_package(autoware_signal_processing REQUIRED)

# Consume the installed validator ABI explicitly. The R003 rear provider must
# never carry a copied PluginInterface or status-message definition.
find_package(autoware_test_utils REQUIRED CONFIG)
find_package(catkin REQUIRED COMPONENTS
  autoware_map_msgs
  autoware_lanelet2_utils
  autoware_motion_utils
  autoware_perception_msgs
  autoware_planning_msgs
  autoware_utils_geometry
  autoware_vehicle_info_utils
  geometry_msgs
  nav_msgs
  pluginlib
  roscpp
  sensor_msgs
  tf2_ros
)
find_package(PCL REQUIRED COMPONENTS common filters kdtree segmentation surface)
find_package(autoware_planning_validator REQUIRED CONFIG)
get_filename_component(AW149_VALIDATOR_PREFIX "${autoware_planning_validator_DIR}/../../.." ABSOLUTE)
find_package(autoware_route_handler REQUIRED CONFIG)
get_filename_component(AW149_ROUTE_HANDLER_PREFIX "${autoware_route_handler_DIR}/../../.." ABSOLUTE)
set(AW149_ROUTE_FIXTURE_PATH
  "${autoware_test_utils_DIR}/../test_map/overlap_map.osm"
  CACHE FILEPATH "Frozen RouteHandler fixture used by the R003 provider oracle")
foreach(abi_header
    include/autoware/planning_validator/plugin_interface.hpp
    include/autoware/planning_validator/trajectory_checker_core.hpp
    include/autoware_planning_validator/PlanningValidatorStatus.h)
  if(NOT EXISTS "${AW149_VALIDATOR_PREFIX}/${abi_header}")
    message(FATAL_ERROR "Missing required validator ABI artifact: ${AW149_VALIDATOR_PREFIX}/${abi_header}")
  endif()
endforeach()
if(NOT EXISTS "${AW149_ROUTE_HANDLER_PREFIX}/include/autoware/route_handler/route_handler.hpp")
  message(FATAL_ERROR "Missing required RouteHandler ABI header under ${AW149_ROUTE_HANDLER_PREFIX}/include")
endif()
unset(AW149_VALIDATOR_MANAGER_LIBRARY CACHE)
find_library(AW149_VALIDATOR_MANAGER_LIBRARY
  NAMES planning_validator_manager_ros1
  PATHS "${AW149_VALIDATOR_PREFIX}/lib"
  NO_DEFAULT_PATH
)
if(NOT AW149_VALIDATOR_MANAGER_LIBRARY)
  message(FATAL_ERROR "Missing PlanningValidatorManager library under ${AW149_VALIDATOR_PREFIX}/lib")
endif()
unset(AW149_REAR_POINTCLOUD_CORE_LIBRARY CACHE)
find_library(AW149_REAR_POINTCLOUD_CORE_LIBRARY
  NAMES rear_pointcloud_core_ros1
  PATHS "${AW149_VALIDATOR_PREFIX}/lib"
  NO_DEFAULT_PATH
)
unset(AW149_REAR_POINTCLOUD_TF_CORE_LIBRARY CACHE)
find_library(AW149_REAR_POINTCLOUD_TF_CORE_LIBRARY
  NAMES rear_pointcloud_tf_core_ros1
  PATHS "${AW149_VALIDATOR_PREFIX}/lib"
  NO_DEFAULT_PATH
)
unset(AW149_ROUTE_HANDLER_LIBRARY CACHE)
find_library(AW149_ROUTE_HANDLER_LIBRARY
  NAMES autoware_route_handler
  PATHS "${AW149_ROUTE_HANDLER_PREFIX}/lib"
  NO_DEFAULT_PATH
)
if(NOT AW149_ROUTE_HANDLER_LIBRARY)
  message(FATAL_ERROR "Missing RouteHandler ABI library under ${AW149_ROUTE_HANDLER_PREFIX}/lib")
endif()
foreach(rear_core
    REAR_CURRENT_LANE_CORE_LIBRARY
    REAR_COLLISION_DECISION_CORE_LIBRARY
    REAR_LANELET_DETECTION_CORE_LIBRARY
    REAR_MOTION_ENVELOPE_CORE_LIBRARY
    REAR_OBJECT_ASSEMBLY_CORE_LIBRARY
    REAR_OBJECT_TRACKING_CORE_LIBRARY
    REAR_SHIFT_ACTIVATION_CORE_LIBRARY
    REAR_TURN_ACTIVATION_CORE_LIBRARY)
  string(REPLACE "_LIBRARY" "" rear_core_stem "${rear_core}")
  string(TOLOWER "${rear_core_stem}" rear_core_file)
  find_library(AW149_${rear_core}
    NAMES ${rear_core_file}_ros1
    PATHS "${AW149_VALIDATOR_PREFIX}/lib"
    NO_DEFAULT_PATH)
endforeach()
if(NOT AW149_REAR_POINTCLOUD_CORE_LIBRARY OR NOT AW149_REAR_POINTCLOUD_TF_CORE_LIBRARY OR
   NOT AW149_REAR_CURRENT_LANE_CORE_LIBRARY OR NOT AW149_REAR_MOTION_ENVELOPE_CORE_LIBRARY OR
   NOT AW149_REAR_COLLISION_DECISION_CORE_LIBRARY OR
   NOT AW149_REAR_LANELET_DETECTION_CORE_LIBRARY OR
   NOT AW149_REAR_OBJECT_ASSEMBLY_CORE_LIBRARY OR
   NOT AW149_REAR_OBJECT_TRACKING_CORE_LIBRARY OR
   NOT AW149_REAR_SHIFT_ACTIVATION_CORE_LIBRARY OR
   NOT AW149_REAR_TURN_ACTIVATION_CORE_LIBRARY)
  message(FATAL_ERROR "Missing installed R003 validator core libraries under ${AW149_VALIDATOR_PREFIX}/lib")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_full_source ${PROJECT_NAME}
  CATKIN_DEPENDS autoware_lanelet2_utils autoware_motion_utils autoware_utils_geometry pluginlib roscpp
  DEPENDS PCL
)

include_directories(
  include
  "${AW149_VALIDATOR_PREFIX}/include"
  "${AW149_ROUTE_HANDLER_PREFIX}/include"
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME} SHARED src/rear_collision_checker_plugin.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${AW149_REAR_POINTCLOUD_CORE_LIBRARY}
  ${AW149_REAR_POINTCLOUD_TF_CORE_LIBRARY}
  ${AW149_REAR_CURRENT_LANE_CORE_LIBRARY}
  ${AW149_REAR_COLLISION_DECISION_CORE_LIBRARY}
  ${AW149_REAR_LANELET_DETECTION_CORE_LIBRARY}
  ${AW149_REAR_MOTION_ENVELOPE_CORE_LIBRARY}
  ${AW149_REAR_OBJECT_ASSEMBLY_CORE_LIBRARY}
  ${AW149_REAR_OBJECT_TRACKING_CORE_LIBRARY}
  ${AW149_REAR_SHIFT_ACTIVATION_CORE_LIBRARY}
  ${AW149_REAR_TURN_ACTIVATION_CORE_LIBRARY}
  ${AW149_ROUTE_HANDLER_LIBRARY}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

if(CATKIN_ENABLE_TESTING)
  if(NOT EXISTS "${AW149_ROUTE_FIXTURE_PATH}")
    message(FATAL_ERROR "Missing R003 RouteHandler oracle fixture: ${AW149_ROUTE_FIXTURE_PATH}")
  endif()
  catkin_add_gtest(rear_collision_checker_provider_oracle test/rear_collision_checker_provider_oracle.cpp)
  if(TARGET rear_collision_checker_provider_oracle)
    add_dependencies(rear_collision_checker_provider_oracle ${catkin_EXPORTED_TARGETS})
    target_link_libraries(rear_collision_checker_provider_oracle ${PROJECT_NAME} ${catkin_LIBRARIES})
    target_compile_definitions(rear_collision_checker_provider_oracle PRIVATE
      AW149_ROUTE_FIXTURE_PATH="${AW149_ROUTE_FIXTURE_PATH}")
    target_compile_features(rear_collision_checker_provider_oracle PUBLIC cxx_std_17)
  endif()
  catkin_add_gtest(rear_collision_checker_pluginlib_oracle test/rear_collision_checker_pluginlib_oracle.cpp)
  if(TARGET rear_collision_checker_pluginlib_oracle)
    add_dependencies(rear_collision_checker_pluginlib_oracle ${catkin_EXPORTED_TARGETS})
    target_link_libraries(rear_collision_checker_pluginlib_oracle ${catkin_LIBRARIES})
    target_compile_features(rear_collision_checker_pluginlib_oracle PUBLIC cxx_std_17)
  endif()
  catkin_add_gtest(rear_collision_checker_manager_oracle test/rear_collision_checker_manager_oracle.cpp)
  if(TARGET rear_collision_checker_manager_oracle)
    add_dependencies(rear_collision_checker_manager_oracle ${catkin_EXPORTED_TARGETS})
    target_link_libraries(rear_collision_checker_manager_oracle ${AW149_VALIDATOR_MANAGER_LIBRARY} ${catkin_LIBRARIES})
    target_compile_features(rear_collision_checker_manager_oracle PUBLIC cxx_std_17)
  endif()
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(FILES package.xml plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

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
add_library(${PROJECT_NAME}_full_source src/full_source/rear_collision_checker.cpp src/full_source/utils.cpp)
target_compile_features(${PROJECT_NAME}_full_source PUBLIC cxx_std_17)
add_dependencies(${PROJECT_NAME}_full_source ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_full_source ${autoware_planning_validator_LIBRARIES}
  ${autoware_signal_processing_LIBRARIES}
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${autoware_utils_LIBRARIES}
  ${autoware_utils_debug_LIBRARIES} ${autoware_utils_logging_LIBRARIES}
  ${autoware_planning_factor_interface_LIBRARIES} ${diagnostic_updater_LIBRARIES})
set_target_properties(${PROJECT_NAME}_full_source PROPERTIES LINK_FLAGS "-Wl,--no-undefined")
install(TARGETS ${PROJECT_NAME}_full_source
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES full_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
