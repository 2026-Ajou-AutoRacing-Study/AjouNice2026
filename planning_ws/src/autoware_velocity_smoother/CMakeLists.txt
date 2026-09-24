cmake_minimum_required(VERSION 3.0.2)
project(autoware_velocity_smoother)

find_package(catkin REQUIRED COMPONENTS
  roscpp geometry_msgs nav_msgs visualization_msgs tf2 tf2_geometry_msgs
  autoware_adapi_v1_msgs autoware_internal_debug_msgs autoware_internal_planning_msgs
  autoware_planning_msgs autoware_motion_utils autoware_trajectory
  autoware_osqp_interface autoware_qp_interface autoware_utils_debug
  autoware_utils_diagnostics autoware_utils_geometry autoware_utils_logging
  autoware_utils_math autoware_utils_rclcpp autoware_utils_system
  autoware_vehicle_info_utils)
find_package(Eigen3 REQUIRED)

# The inherited base workspace contains an older autoware_planning_msgs header
# with a different ROS1 MD5.  Resolve this candidate's declared provider first
# so its node ABI matches the message package exported by its runtime underlay.
find_path(AUTOWARE_PLANNING_MSGS_AUTHORITATIVE_INCLUDE
  NAMES autoware_planning_msgs/Trajectory.h
  PATHS ${autoware_planning_msgs_DIR}/../../..
  PATH_SUFFIXES include
  NO_DEFAULT_PATH
)
if(NOT AUTOWARE_PLANNING_MSGS_AUTHORITATIVE_INCLUDE)
  message(FATAL_ERROR "Declared autoware_planning_msgs provider lacks Trajectory.h")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES autoware_velocity_smoother
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs visualization_msgs tf2 tf2_geometry_msgs
    autoware_adapi_v1_msgs autoware_internal_debug_msgs autoware_internal_planning_msgs
    autoware_planning_msgs autoware_motion_utils autoware_trajectory autoware_osqp_interface
    autoware_qp_interface autoware_utils_debug autoware_utils_diagnostics
    autoware_utils_geometry autoware_utils_logging autoware_utils_math autoware_utils_rclcpp
    autoware_utils_system autoware_vehicle_info_utils)

set(SMOOTHER_SRC
  src/smoother/smoother_base.cpp
  src/smoother/l2_pseudo_jerk_smoother.cpp
  src/smoother/linf_pseudo_jerk_smoother.cpp
  src/smoother/jerk_filtered_smoother.cpp
  src/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.cpp
  src/smoother/analytical_jerk_constrained_smoother/velocity_planning_utils.cpp
  src/trajectory_utils.cpp
  src/resample.cpp
)

add_library(${PROJECT_NAME} SHARED ${SMOOTHER_SRC})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} BEFORE PUBLIC
  ${AUTOWARE_PLANNING_MSGS_AUTHORITATIVE_INCLUDE})
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} BEFORE PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} Eigen3::Eigen)

set(AW149_VELOCITY_SMOOTHER_INSTALL_TARGETS ${PROJECT_NAME})
# The export is a runnable source deliverable, not a library-only staging port.
# Compile/install the real node unconditionally so missing runtime ABI support
# is exposed by a normal workspace build rather than hidden behind an OFF flag.
  add_executable(velocity_smoother_node src/node.cpp src/main.cpp)
  target_compile_features(velocity_smoother_node PUBLIC cxx_std_17)
  target_include_directories(velocity_smoother_node BEFORE PRIVATE
    ${AUTOWARE_PLANNING_MSGS_AUTHORITATIVE_INCLUDE})
  target_include_directories(velocity_smoother_node PRIVATE include ${catkin_INCLUDE_DIRS})
  target_include_directories(velocity_smoother_node BEFORE PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(velocity_smoother_node PRIVATE ${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen)
  list(APPEND AW149_VELOCITY_SMOOTHER_INSTALL_TARGETS velocity_smoother_node)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_velocity_smoother_trajectory_utils
    test/test_main.cpp
    test/test_smoother_functions.cpp)
  if(TARGET test_velocity_smoother_trajectory_utils)
    target_include_directories(test_velocity_smoother_trajectory_utils BEFORE PRIVATE
      ${AUTOWARE_PLANNING_MSGS_AUTHORITATIVE_INCLUDE})
    target_link_libraries(test_velocity_smoother_trajectory_utils
      ${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen)
  endif()

  catkin_add_gtest(test_velocity_smoother_resample
    test/test_main.cpp
    test/test_resample.cpp)
  if(TARGET test_velocity_smoother_resample)
    target_include_directories(test_velocity_smoother_resample BEFORE PRIVATE
      ${AUTOWARE_PLANNING_MSGS_AUTHORITATIVE_INCLUDE})
    target_include_directories(test_velocity_smoother_resample BEFORE PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(test_velocity_smoother_resample
      ${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen)
  endif()

  catkin_add_gtest(test_velocity_smoother_stop_velocity
    test/test_stop_velocity_calculation.cpp)
  if(TARGET test_velocity_smoother_stop_velocity)
    target_include_directories(test_velocity_smoother_stop_velocity BEFORE PRIVATE
      ${AUTOWARE_PLANNING_MSGS_AUTHORITATIVE_INCLUDE})
    target_link_libraries(test_velocity_smoother_stop_velocity
      ${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen)
  endif()
endif()

install(TARGETS ${AW149_VELOCITY_SMOOTHER_INSTALL_TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/autoware/velocity_smoother/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/autoware/velocity_smoother)
# The staged root config/launch assets still use ROS2 syntax. Only the explicit
# ROS1 rosparam subset is installable; remaining algorithm variants and dynamic
# reconfiguration stay blocked pending their dedicated equivalence work.
install(DIRECTORY config/ros1/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config/ros1)
install(FILES launch/velocity_smoother_ros1.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
