cmake_minimum_required(VERSION 3.0.2)
project(trajectory_generator)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  autoware_planning_msgs
  geometry_msgs
  nav_msgs
  roscpp
  roslib
  tf2
  tf2_geometry_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS
    autoware_planning_msgs
    geometry_msgs
    nav_msgs
    roscpp
    roslib
    tf2
    tf2_geometry_msgs
    visualization_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(trajectory_publisher_node src/trajectory_publisher_node.cpp)
add_dependencies(trajectory_publisher_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_publisher_node ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/generate_constant_curvature_lc.py
  scripts/generate_constant_curvature_return.py
  scripts/generate_trajectory.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS trajectory_publisher_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch traj_sample
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
