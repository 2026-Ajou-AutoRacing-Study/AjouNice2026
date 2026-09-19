cmake_minimum_required(VERSION 3.0.2)
project(autoware_planning_validator_latency_checker)

# The validator ABI is deliberately consumed from the R002 candidate install,
# rather than copied into this provider.  Do not call find_package on the
# validator's full catkin export here: that export recursively pulls every
# route/map implementation even though this provider only consumes the plugin
# ABI and generated status message.  The explicit header checks below retain a
# real ABI boundary without accidentally accepting a transitive closure.
find_package(catkin REQUIRED COMPONENTS
  autoware_map_msgs
  autoware_perception_msgs
  autoware_planning_msgs
  autoware_vehicle_info_utils
  geometry_msgs
  nav_msgs
  pluginlib
  roscpp
  sensor_msgs
  tf2_ros
)
find_package(autoware_planning_validator REQUIRED CONFIG)
get_filename_component(AW149_VALIDATOR_PREFIX "${autoware_planning_validator_DIR}/../../.." ABSOLUTE)
foreach(abi_header
    include/autoware/planning_validator/plugin_interface.hpp
    include/autoware/planning_validator/trajectory_checker_core.hpp
    include/autoware_planning_validator/PlanningValidatorStatus.h)
  if(NOT EXISTS "${AW149_VALIDATOR_PREFIX}/${abi_header}")
    message(FATAL_ERROR "Missing required R002 validator ABI artifact: ${AW149_VALIDATOR_PREFIX}/${abi_header}")
  endif()
endforeach()
unset(AW149_VALIDATOR_MANAGER_LIBRARY CACHE)
find_library(AW149_VALIDATOR_MANAGER_LIBRARY
  NAMES planning_validator_manager_ros1
  PATHS "${AW149_VALIDATOR_PREFIX}/lib"
  NO_DEFAULT_PATH
)
if(NOT AW149_VALIDATOR_MANAGER_LIBRARY)
  message(FATAL_ERROR "Missing R002 PlanningValidatorManager library under ${AW149_VALIDATOR_PREFIX}/lib")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_full_source ${PROJECT_NAME}
  CATKIN_DEPENDS pluginlib roscpp
)

include_directories(include "${AW149_VALIDATOR_PREFIX}/include" ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED src/latency_checker.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(latency_checker_provider_oracle test/latency_checker_provider_oracle.cpp)
  if(TARGET latency_checker_provider_oracle)
    add_dependencies(latency_checker_provider_oracle ${catkin_EXPORTED_TARGETS})
    target_link_libraries(latency_checker_provider_oracle ${PROJECT_NAME} ${catkin_LIBRARIES})
    target_compile_features(latency_checker_provider_oracle PUBLIC cxx_std_17)
  endif()
  catkin_add_gtest(latency_checker_pluginlib_oracle test/latency_checker_pluginlib_oracle.cpp)
  if(TARGET latency_checker_pluginlib_oracle)
    add_dependencies(latency_checker_pluginlib_oracle ${catkin_EXPORTED_TARGETS})
    target_link_libraries(latency_checker_pluginlib_oracle ${catkin_LIBRARIES})
    target_compile_features(latency_checker_pluginlib_oracle PUBLIC cxx_std_17)
    set_tests_properties(
      _ctest_${PROJECT_NAME}_gtest_latency_checker_pluginlib_oracle
      PROPERTIES ENVIRONMENT
      "ROS_PACKAGE_PATH=${CMAKE_INSTALL_PREFIX}/share:${AW149_VALIDATOR_PREFIX}/share:$ENV{ROS_PACKAGE_PATH};CMAKE_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:${AW149_VALIDATOR_PREFIX}:$ENV{CMAKE_PREFIX_PATH};LD_LIBRARY_PATH=${CMAKE_INSTALL_PREFIX}/lib:${AW149_VALIDATOR_PREFIX}/lib:$ENV{LD_LIBRARY_PATH}")
  endif()
  catkin_add_gtest(latency_checker_manager_oracle test/latency_checker_manager_oracle.cpp)
  if(TARGET latency_checker_manager_oracle)
    add_dependencies(latency_checker_manager_oracle ${catkin_EXPORTED_TARGETS})
    target_link_libraries(latency_checker_manager_oracle ${AW149_VALIDATOR_MANAGER_LIBRARY} ${catkin_LIBRARIES})
    target_compile_features(latency_checker_manager_oracle PUBLIC cxx_std_17)
    set_tests_properties(
      _ctest_${PROJECT_NAME}_gtest_latency_checker_manager_oracle
      PROPERTIES ENVIRONMENT
      "ROS_PACKAGE_PATH=${CMAKE_INSTALL_PREFIX}/share:${AW149_VALIDATOR_PREFIX}/share:$ENV{ROS_PACKAGE_PATH};CMAKE_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:${AW149_VALIDATOR_PREFIX}:$ENV{CMAKE_PREFIX_PATH};LD_LIBRARY_PATH=${CMAKE_INSTALL_PREFIX}/lib:${AW149_VALIDATOR_PREFIX}/lib:$ENV{LD_LIBRARY_PATH}")
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
add_library(${PROJECT_NAME}_full_source src/full_source/latency_checker.cpp)
target_compile_features(${PROJECT_NAME}_full_source PUBLIC cxx_std_17)
add_dependencies(${PROJECT_NAME}_full_source ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_full_source ${autoware_planning_validator_LIBRARIES}
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${autoware_utils_LIBRARIES}
  ${autoware_utils_debug_LIBRARIES} ${autoware_utils_logging_LIBRARIES}
  ${autoware_planning_factor_interface_LIBRARIES} ${diagnostic_updater_LIBRARIES})
set_target_properties(${PROJECT_NAME}_full_source PROPERTIES LINK_FLAGS "-Wl,--no-undefined")
install(TARGETS ${PROJECT_NAME}_full_source
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES full_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
