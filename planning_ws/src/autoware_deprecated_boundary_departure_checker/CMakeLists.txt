cmake_minimum_required(VERSION 3.0.2)
project(autoware_deprecated_boundary_departure_checker)

find_package(catkin REQUIRED COMPONENTS
  autoware_interpolation
  autoware_internal_planning_msgs
  autoware_motion_utils
  autoware_planning_msgs
  autoware_trajectory
  autoware_utils_debug
  autoware_utils_geometry
  autoware_utils_math
  autoware_utils_system
  autoware_utils_uuid
  autoware_utils_visualization
  autoware_vehicle_info_utils
  autoware_vehicle_msgs
  builtin_interfaces
  geometry_msgs
  nav_msgs
  roscpp
  tf2
  tf2_geometry_msgs
  visualization_msgs
)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(tl_expected REQUIRED)

# Modern CMake packages expose imported targets rather than the legacy
# variables catkin_package() uses to generate its install-time dependency
# config.  Export both forms so downstream ROS1 consumers do not rely on an
# accidental transitive include path from an overlay.
get_target_property(Eigen3_INCLUDE_DIRS Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
set(Eigen3_LIBRARIES Eigen3::Eigen)
get_target_property(fmt_INCLUDE_DIRS fmt::fmt INTERFACE_INCLUDE_DIRECTORIES)
set(fmt_LIBRARIES fmt::fmt)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES boundary_departure_checker_utils uncrossable_boundary_departure_checker boundary_departure_checker
  CATKIN_DEPENDS autoware_interpolation autoware_internal_planning_msgs autoware_motion_utils
    autoware_planning_msgs autoware_trajectory autoware_utils_debug
    autoware_utils_geometry autoware_utils_math autoware_utils_system autoware_utils_uuid
    autoware_utils_visualization autoware_vehicle_info_utils autoware_vehicle_msgs
    builtin_interfaces geometry_msgs nav_msgs roscpp tf2 tf2_geometry_msgs visualization_msgs
  DEPENDS Eigen3 fmt lanelet2_core tl_expected
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(boundary_departure_checker_utils SHARED
  src/conversion.cpp
  src/steering_abnormality_utils.cpp
  src/utils.cpp
  src/debug.cpp
  src/footprint_generator/normal.cpp
  src/footprint_generator/localization.cpp
  src/footprint_generator/longitudinal.cpp
  src/footprint_generator/steering.cpp
  src/footprint_generator/manager.cpp
)

add_library(uncrossable_boundary_departure_checker SHARED
  src/uncrossable_boundary_checker.cpp
)

target_link_libraries(uncrossable_boundary_departure_checker boundary_departure_checker_utils ${catkin_LIBRARIES})

add_library(boundary_departure_checker SHARED
  src/boundary_departure_checker.cpp
)

target_link_libraries(boundary_departure_checker boundary_departure_checker_utils ${catkin_LIBRARIES})

foreach(target boundary_departure_checker_utils uncrossable_boundary_departure_checker boundary_departure_checker)
  target_link_libraries(${target} ${catkin_LIBRARIES} Eigen3::Eigen fmt::fmt lanelet2_core::lanelet2_core)
  target_compile_features(${target} PUBLIC cxx_std_17)
endforeach()

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_boundary_departure_checker_ros1 test/test_boundary_departure_checker_ros1.cpp)
  if(TARGET test_boundary_departure_checker_ros1)
    target_link_libraries(test_boundary_departure_checker_ros1
      boundary_departure_checker
      uncrossable_boundary_departure_checker
      ${catkin_LIBRARIES})
    target_compile_features(test_boundary_departure_checker_ros1 PRIVATE cxx_std_17)
  endif()
endif()

install(TARGETS boundary_departure_checker_utils uncrossable_boundary_departure_checker boundary_departure_checker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(FILES package.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
