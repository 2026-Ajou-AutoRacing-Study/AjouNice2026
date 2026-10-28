cmake_minimum_required(VERSION 3.0.2)
project(autoware_behavior_path_dynamic_obstacle_avoidance_module)

# This candidate may only consume explicitly staged ROS1 providers.  In
# particular, never allow a ROS2 overlay to satisfy an include or library.
find_package(catkin REQUIRED COMPONENTS geometry_msgs pluginlib roscpp visualization_msgs)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(autoware_utils_rclcpp REQUIRED)

include("${CMAKE_CURRENT_LIST_DIR}/../cmake/AW149Providers.cmake")
aw149_provider(BPP_COMMON autoware_behavior_path_planner_common autoware_behavior_path_planner_common)
aw149_provider(OBJECT_RECOGNITION_UTILS autoware_object_recognition_utils autoware_object_recognition_utils)
aw149_provider(SIGNAL_PROCESSING autoware_signal_processing lowpass_filters)
aw149_provider(PERCEPTION_MSGS autoware_perception_msgs NONE)
aw149_provider(INTERNAL_PLANNING_MSGS autoware_internal_planning_msgs NONE)
aw149_provider(VEHICLE_MSGS autoware_vehicle_msgs NONE)
aw149_provider(TIER4_PLANNING_MSGS tier4_planning_msgs NONE)
set(BPP_TRANSITIVE_INCLUDE_DIRS "${autoware_behavior_path_planner_common_INCLUDE_DIRS}" CACHE STRING "Exported BPP dependency includes" FORCE)
set(BPP_RUNTIME_LIBRARIES "${autoware_behavior_path_planner_common_LIBRARIES}" CACHE STRING "Exported BPP dependency libraries" FORCE)

foreach(provider IN ITEMS
  BPP_COMMON_INCLUDE_DIR BPP_COMMON_LIBRARY
  BPP_TRANSITIVE_INCLUDE_DIRS BPP_RUNTIME_LIBRARIES
  OBJECT_RECOGNITION_UTILS_INCLUDE_DIR OBJECT_RECOGNITION_UTILS_LIBRARY
  SIGNAL_PROCESSING_INCLUDE_DIR SIGNAL_PROCESSING_LIBRARY
  PERCEPTION_MSGS_INCLUDE_DIR INTERNAL_PLANNING_MSGS_INCLUDE_DIR
  VEHICLE_MSGS_INCLUDE_DIR TIER4_PLANNING_MSGS_INCLUDE_DIR)
  set(${provider} "" CACHE PATH "staged Dynamic Obstacle Avoidance ROS1 provider")
  if("${provider}" STREQUAL "BPP_TRANSITIVE_INCLUDE_DIRS" OR "${provider}" STREQUAL "BPP_RUNTIME_LIBRARIES")
    if("${${provider}}" STREQUAL "")
      message(FATAL_ERROR "Set ${provider} to the exact staged BPP ROS1 runtime closure")
    endif()
  elseif(NOT EXISTS "${${provider}}")
    message(FATAL_ERROR "Set ${provider} to its staged ROS1 provider; ambient discovery is forbidden")
  endif()
endforeach()

catkin_package(LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS autoware_utils_rclcpp geometry_msgs pluginlib roscpp visualization_msgs)

include_directories(
  include ${autoware_utils_rclcpp_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
  ${BPP_COMMON_INCLUDE_DIR}
  ${BPP_TRANSITIVE_INCLUDE_DIRS}
  ${OBJECT_RECOGNITION_UTILS_INCLUDE_DIR} ${SIGNAL_PROCESSING_INCLUDE_DIR}
  ${PERCEPTION_MSGS_INCLUDE_DIR} ${INTERNAL_PLANNING_MSGS_INCLUDE_DIR}
  ${VEHICLE_MSGS_INCLUDE_DIR} ${TIER4_PLANNING_MSGS_INCLUDE_DIR})

add_library(${PROJECT_NAME} SHARED src/scene.cpp src/manager.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
# The provider closure is a runtime contract, not an implementation detail of
# BPP common.  Retain every staged dependency in this plugin's NEEDED set.
set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-Wl,--no-as-needed")
target_link_libraries(${PROJECT_NAME}
  ${BPP_COMMON_LIBRARY} ${BPP_RUNTIME_LIBRARIES}
  ${OBJECT_RECOGNITION_UTILS_LIBRARY} ${SIGNAL_PROCESSING_LIBRARY}
  fmt::fmt ${lanelet2_core_LIBRARIES} ${catkin_LIBRARIES})

add_executable(aw149_doa_plugin_discovery test/ros1_plugin_discovery.cpp)
target_compile_features(aw149_doa_plugin_discovery PUBLIC cxx_std_17)
target_link_libraries(aw149_doa_plugin_discovery ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(aw149_doa_manager_init test/ros1_manager_init.cpp)
target_compile_features(aw149_doa_manager_init PUBLIC cxx_std_17)
target_link_libraries(aw149_doa_manager_init ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(aw149_doa_manager_config_init test/ros1_manager_config_init.cpp)
target_compile_features(aw149_doa_manager_config_init PUBLIC cxx_std_17)
target_link_libraries(aw149_doa_manager_config_init ${PROJECT_NAME} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_doa_independent_constraints test/test_independent_constraints.cpp)
  if(TARGET test_doa_independent_constraints)
    target_compile_features(test_doa_independent_constraints PUBLIC cxx_std_17)
    target_link_libraries(test_doa_independent_constraints ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION include)
install(FILES package.xml plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
