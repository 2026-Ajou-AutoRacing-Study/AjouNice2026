cmake_minimum_required(VERSION 3.0.2)
project(autoware_motion_velocity_boundary_departure_prevention_module)

# R002 keeps the complete upstream source surface. Provider closure is made
# explicit here; no source unit may be silently dropped to obtain a build.
find_package(catkin REQUIRED COMPONENTS
  autoware_adapi_v1_msgs
  autoware_control_msgs
  autoware_deprecated_boundary_departure_checker
  autoware_motion_utils
  autoware_motion_velocity_planner_common
  autoware_object_recognition_utils
  autoware_route_handler
  autoware_planning_msgs
  autoware_trajectory
  autoware_velocity_smoother
  autoware_vehicle_msgs
  diagnostic_updater
  geometry_msgs
  nav_msgs
  pluginlib
  roscpp
  tf2
  visualization_msgs
)
# The planner-common ABI exposes velocity-smoother symbols but its exported
# catkin dependency closure omits the corresponding DSO.  Keep this candidate
# self-contained instead of relying on loader order from another module.
find_package(autoware_velocity_smoother REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS autoware_adapi_v1_msgs autoware_control_msgs
    autoware_deprecated_boundary_departure_checker autoware_motion_utils
    autoware_motion_velocity_planner_common autoware_object_recognition_utils
    autoware_route_handler autoware_planning_msgs autoware_trajectory autoware_velocity_smoother autoware_vehicle_msgs
    diagnostic_updater geometry_msgs nav_msgs pluginlib roscpp tf2 visualization_msgs
)

include_directories(src ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} SHARED
  src/boundary_departure_prevention_module.cpp
  src/debug.cpp
  src/slow_down_interpolator.cpp
  src/utils.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${autoware_velocity_smoother_LIBRARIES})
# A plugin may otherwise be linked with --as-needed and appear to build while
# losing its provider DSOs from DT_NEEDED.  Keep every explicit catkin provider
# and reject unresolved ABI symbols at link time, before pluginlib can load it.
target_link_options(${PROJECT_NAME} PRIVATE -Wl,--no-as-needed -Wl,-z,defs)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

add_executable(bdp_pluginlib_load_oracle test/pluginlib_load_oracle.cpp)
target_link_libraries(bdp_pluginlib_load_oracle ${PROJECT_NAME} ${catkin_LIBRARIES} ${autoware_velocity_smoother_LIBRARIES})
target_compile_features(bdp_pluginlib_load_oracle PRIVATE cxx_std_17)

add_executable(bdp_input_gate_oracle test/input_gate_oracle.cpp)
target_link_libraries(bdp_input_gate_oracle ${PROJECT_NAME} ${catkin_LIBRARIES} ${autoware_velocity_smoother_LIBRARIES})
target_compile_features(bdp_input_gate_oracle PRIVATE cxx_std_17)

add_executable(bdp_planning_fixture_oracle test/planning_fixture_oracle.cpp)
target_link_libraries(bdp_planning_fixture_oracle ${PROJECT_NAME} ${catkin_LIBRARIES} ${autoware_velocity_smoother_LIBRARIES})
target_compile_features(bdp_planning_fixture_oracle PRIVATE cxx_std_17)

if(CATKIN_ENABLE_TESTING)
  # Both are self-contained decision oracles; pluginlib loading remains a
  # separate installed-package gate because it depends on ROS package lookup.
  add_test(NAME bdp_input_gate_oracle COMMAND $<TARGET_FILE:bdp_input_gate_oracle>)
  add_test(NAME bdp_planning_fixture_oracle COMMAND $<TARGET_FILE:bdp_planning_fixture_oracle>)
endif()

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(FILES package.xml plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
