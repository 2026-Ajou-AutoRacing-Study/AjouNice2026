cmake_minimum_required(VERSION 3.0.2)
project(autoware_planning_validator_trajectory_checker)

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
foreach(abi_artifact
    include/autoware/planning_validator/plugin_interface.hpp
    include/autoware/planning_validator/trajectory_checker_core.hpp
    include/autoware_planning_validator/PlanningValidatorStatus.h
    lib/libtrajectory_checker_core_ros1.so
    lib/libplanning_validator_manager_ros1.so)
  if(NOT EXISTS "${AW149_VALIDATOR_PREFIX}/${abi_artifact}")
    message(FATAL_ERROR "Missing required R002 validator ABI artifact: ${AW149_VALIDATOR_PREFIX}/${abi_artifact}")
  endif()
endforeach()
unset(AW149_TRAJECTORY_CORE_LIBRARY CACHE)
find_library(AW149_TRAJECTORY_CORE_LIBRARY NAMES trajectory_checker_core_ros1 PATHS "${AW149_VALIDATOR_PREFIX}/lib" NO_DEFAULT_PATH)
unset(AW149_VALIDATOR_MANAGER_LIBRARY CACHE)
find_library(AW149_VALIDATOR_MANAGER_LIBRARY NAMES planning_validator_manager_ros1 PATHS "${AW149_VALIDATOR_PREFIX}/lib" NO_DEFAULT_PATH)
if(NOT AW149_TRAJECTORY_CORE_LIBRARY OR NOT AW149_VALIDATOR_MANAGER_LIBRARY)
  message(FATAL_ERROR "R002 validator provider libraries are not discoverable")
endif()

catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME}_full_source ${PROJECT_NAME} CATKIN_DEPENDS pluginlib roscpp)
include_directories(include "${AW149_VALIDATOR_PREFIX}/include" ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED src/trajectory_checker.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${AW149_TRAJECTORY_CORE_LIBRARY} ${catkin_LIBRARIES})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

if(CATKIN_ENABLE_TESTING)
  # The trajectory provider is deliberately thin: it consumes the accepted
  # planning-validator core ABI, whose DSO closure reaches motion-utils and
  # the route/lanelet/trajectory providers.  Keep the manager oracle hermetic
  # by declaring that same candidate-only closure instead of depending on an
  # ambient workspace LD_LIBRARY_PATH.
  set(AW149_PROVIDER_RUNTIME_LD_LIBRARY_PATH "${CMAKE_INSTALL_PREFIX}/lib:$ENV{LD_LIBRARY_PATH}")
  catkin_add_gtest(trajectory_checker_manager_oracle test/trajectory_checker_manager_oracle.cpp)
  if(TARGET trajectory_checker_manager_oracle)
    add_dependencies(trajectory_checker_manager_oracle ${catkin_EXPORTED_TARGETS})
    target_link_libraries(trajectory_checker_manager_oracle ${AW149_VALIDATOR_MANAGER_LIBRARY} ${catkin_LIBRARIES})
    target_compile_features(trajectory_checker_manager_oracle PUBLIC cxx_std_17)
    set_tests_properties(
      _ctest_${PROJECT_NAME}_gtest_trajectory_checker_manager_oracle
      PROPERTIES ENVIRONMENT
      "ROS_PACKAGE_PATH=${CMAKE_INSTALL_PREFIX}/share:${AW149_VALIDATOR_PREFIX}/share:$ENV{ROS_PACKAGE_PATH};CMAKE_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:${AW149_VALIDATOR_PREFIX}:$ENV{CMAKE_PREFIX_PATH};LD_LIBRARY_PATH=${AW149_PROVIDER_RUNTIME_LD_LIBRARY_PATH}")
  endif()
  add_executable(trajectory_checker_config_oracle test/trajectory_checker_config_oracle.cpp)
  add_dependencies(trajectory_checker_config_oracle ${catkin_EXPORTED_TARGETS})
  target_link_libraries(trajectory_checker_config_oracle ${AW149_VALIDATOR_MANAGER_LIBRARY} gtest ${catkin_LIBRARIES})
  target_compile_features(trajectory_checker_config_oracle PUBLIC cxx_std_17)
  add_test(
    NAME trajectory_checker_config_clean_oracle
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/test/run_config_oracle_clean.sh
      $<TARGET_FILE:trajectory_checker_config_oracle>
      ${CMAKE_INSTALL_PREFIX}
      ${AW149_VALIDATOR_PREFIX})
endif()

install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
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
add_library(${PROJECT_NAME}_full_source src/full_source/trajectory_checker.cpp src/full_source/utils.cpp)
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

if(CATKIN_ENABLE_TESTING)
  find_package(autoware_planning_validator_test_utils REQUIRED)
  catkin_add_gtest(full_original_trajectory_checker test/full_original_trajectory_checker.cpp)
  if(TARGET full_original_trajectory_checker)
    target_compile_features(full_original_trajectory_checker PUBLIC cxx_std_17)
    target_link_libraries(full_original_trajectory_checker ${PROJECT_NAME}_full_source
      ${autoware_planning_validator_LIBRARIES} ${autoware_planning_validator_test_utils_LIBRARIES}
      ${catkin_LIBRARIES})
  endif()
endif()

if(CATKIN_ENABLE_TESTING)
  find_package(autoware_test_utils REQUIRED)
  catkin_add_gtest(full_original_diagnostics test/full_original_diagnostics.cpp)
  if(TARGET full_original_diagnostics)
    target_compile_features(full_original_diagnostics PUBLIC cxx_std_17)
    target_link_libraries(full_original_diagnostics ${PROJECT_NAME}_full_source
      ${autoware_planning_validator_LIBRARIES} ${autoware_planning_validator_test_utils_LIBRARIES}
      ${autoware_test_utils_LIBRARIES} ${catkin_LIBRARIES})
  endif()
endif()
