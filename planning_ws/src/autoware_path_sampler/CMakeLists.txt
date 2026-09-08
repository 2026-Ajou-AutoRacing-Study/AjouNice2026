cmake_minimum_required(VERSION 3.0.2)
project(autoware_path_sampler)

find_package(catkin REQUIRED COMPONENTS
  autoware_bezier_sampler autoware_frenet_planner autoware_interpolation
  autoware_internal_debug_msgs autoware_motion_utils autoware_perception_msgs autoware_planning_msgs
  autoware_sampler_common autoware_utils autoware_vehicle_info_utils
  geometry_msgs nav_msgs roscpp std_msgs tf2 visualization_msgs
)

catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS autoware_bezier_sampler autoware_frenet_planner autoware_interpolation autoware_internal_debug_msgs autoware_motion_utils autoware_perception_msgs autoware_planning_msgs autoware_sampler_common autoware_utils autoware_vehicle_info_utils geometry_msgs nav_msgs roscpp std_msgs tf2 visualization_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_algorithms
  src/path_generation.cpp src/prepare_inputs.cpp src/utils/trajectory_utils.cpp)
target_link_libraries(${PROJECT_NAME}_algorithms ${catkin_LIBRARIES})
target_compile_features(${PROJECT_NAME}_algorithms PUBLIC cxx_std_17)
add_executable(path_sampler_node src/node.cpp)
target_link_libraries(path_sampler_node ${PROJECT_NAME}_algorithms ${catkin_LIBRARIES})
target_compile_features(path_sampler_node PUBLIC cxx_std_17)
add_executable(path_sampler_runtime_probe test/runtime_probe.cpp src/node.cpp)
target_link_libraries(path_sampler_runtime_probe ${PROJECT_NAME}_algorithms ${catkin_LIBRARIES})
target_compile_features(path_sampler_runtime_probe PUBLIC cxx_std_17)
target_compile_definitions(path_sampler_runtime_probe PRIVATE PATH_SAMPLER_RUNTIME_PROBE)
install(TARGETS ${PROJECT_NAME}_algorithms path_sampler_node
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
