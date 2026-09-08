cmake_minimum_required(VERSION 3.0.2)
project(autoware_motion_utils)

find_package(catkin REQUIRED COMPONENTS
  autoware_internal_planning_msgs
  autoware_interpolation
  autoware_planning_msgs
  autoware_utils_geometry
  autoware_utils_math
  autoware_utils_system
  autoware_utils_visualization
  builtin_interfaces
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES autoware_motion_utils
  CATKIN_DEPENDS autoware_internal_planning_msgs autoware_interpolation autoware_planning_msgs autoware_utils_geometry autoware_utils_math autoware_utils_system autoware_utils_visualization builtin_interfaces geometry_msgs nav_msgs roscpp std_msgs tf2 tf2_geometry_msgs visualization_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

file(GLOB_RECURSE motion_utils_sources src/*.cpp)
add_library(autoware_motion_utils ${motion_utils_sources})
target_link_libraries(autoware_motion_utils ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  add_executable(autoware_motion_utils_runtime_oracle test/runtime_oracle.cpp)
  target_link_libraries(autoware_motion_utils_runtime_oracle autoware_motion_utils ${catkin_LIBRARIES})
endif()

install(TARGETS autoware_motion_utils
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
