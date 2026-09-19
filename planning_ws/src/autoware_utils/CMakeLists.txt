cmake_minimum_required(VERSION 3.0.2)
project(autoware_utils)

find_package(catkin REQUIRED COMPONENTS
  autoware_utils_debug
  autoware_utils_diagnostics
  autoware_utils_geometry
  autoware_utils_logging
  autoware_utils_math
  autoware_utils_pcl
  autoware_utils_rclcpp
  autoware_utils_system
  autoware_utils_tf
  autoware_utils_uuid
  autoware_utils_visualization
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    autoware_utils_debug
    autoware_utils_diagnostics
    autoware_utils_geometry
    autoware_utils_logging
    autoware_utils_math
    autoware_utils_pcl
    autoware_utils_rclcpp
    autoware_utils_system
    autoware_utils_tf
    autoware_utils_uuid
    autoware_utils_visualization
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
