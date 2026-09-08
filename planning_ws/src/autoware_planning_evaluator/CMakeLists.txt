cmake_minimum_required(VERSION 3.13)
project(autoware_planning_evaluator)
find_package(catkin REQUIRED COMPONENTS roscpp roslib autoware_internal_planning_msgs
  autoware_internal_debug_msgs autoware_motion_utils autoware_perception_msgs
  autoware_planning_factor_interface autoware_planning_msgs autoware_route_handler
  autoware_utils autoware_vehicle_info_utils autoware_vehicle_msgs autoware_lanelet2_utils
  autoware_object_recognition_utils geometry_msgs nav_msgs tf2 tf2_ros tier4_metric_msgs)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
set(CMAKE_CXX_STANDARD 17)
catkin_package(INCLUDE_DIRS include LIBRARIES planning_evaluator_node
  CATKIN_DEPENDS roscpp roslib autoware_internal_planning_msgs autoware_internal_debug_msgs
  autoware_motion_utils autoware_perception_msgs autoware_planning_factor_interface
  autoware_planning_msgs autoware_route_handler autoware_utils autoware_vehicle_info_utils
  autoware_vehicle_msgs autoware_lanelet2_utils autoware_object_recognition_utils
  geometry_msgs nav_msgs tf2 tf2_ros tier4_metric_msgs)
add_library(planning_evaluator_node SHARED
  src/planning_evaluator_node.cpp src/motion_evaluator_node.cpp
  src/metrics_calculator.cpp src/metrics_accumulator.cpp src/obstacle_metrics_calculator.cpp
  src/metrics/deviation_metrics.cpp src/metrics/stability_metrics.cpp
  src/metrics/trajectory_metrics.cpp src/metrics/metrics_utils.cpp
  src/metric_accumulators/common_accumulator.cpp
  src/metric_accumulators/blinker_accumulator.cpp
  src/metric_accumulators/planning_factor_accumulator.cpp
  src/metric_accumulators/steer_accuulator.cpp)
target_include_directories(planning_evaluator_node PUBLIC include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(planning_evaluator_node ${catkin_LIBRARIES} yaml-cpp)
target_link_options(planning_evaluator_node PRIVATE -Wl,--no-undefined)
add_executable(planning_evaluator src/ros1_main.cpp)
target_link_libraries(planning_evaluator planning_evaluator_node)
add_executable(motion_evaluator src/ros1_main.cpp)
target_compile_definitions(motion_evaluator PRIVATE AW149_MOTION_EVALUATOR=1)
target_link_libraries(motion_evaluator planning_evaluator_node)
if(CATKIN_ENABLE_TESTING)
  find_package(autoware_test_utils REQUIRED)
  catkin_add_gtest(test_planning_evaluator test/test_planning_evaluator_node.cpp test/ros1_test_main.cpp)
  if(TARGET test_planning_evaluator)
    target_link_libraries(test_planning_evaluator planning_evaluator_node ${autoware_test_utils_LIBRARIES})
    target_include_directories(test_planning_evaluator PRIVATE ${autoware_test_utils_INCLUDE_DIRS})
    add_custom_target(build_original_evaluator_tests ALL DEPENDS test_planning_evaluator)
  endif()
endif()
install(TARGETS planning_evaluator_node planning_evaluator motion_evaluator
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch")
