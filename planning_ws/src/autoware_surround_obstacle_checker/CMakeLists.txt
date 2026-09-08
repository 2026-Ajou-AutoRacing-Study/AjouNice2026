cmake_minimum_required(VERSION 3.0.2)
project(autoware_surround_obstacle_checker)

find_package(catkin REQUIRED COMPONENTS
  roscpp nav_msgs sensor_msgs pcl_conversions tf2 tf2_ros tf2_eigen tf2_geometry_msgs
  geometry_msgs visualization_msgs autoware_motion_utils autoware_utils autoware_utils_logging autoware_utils_geometry
  autoware_internal_debug_msgs autoware_internal_planning_msgs autoware_perception_msgs
  autoware_vehicle_info_utils autoware_planning_factor_interface)
find_package(PCL REQUIRED COMPONENTS common)

catkin_package(INCLUDE_DIRS include parameter_adapter/include LIBRARIES ${PROJECT_NAME}_core ${PROJECT_NAME}_full_source)
include_directories(include parameter_adapter/include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_core src/surround_obstacle_checker_ros1_node.cpp)
target_link_libraries(${PROJECT_NAME}_core ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)

add_executable(aw149_surround_obstacle_checker_node src/main.cpp)
target_link_libraries(aw149_surround_obstacle_checker_node ${PROJECT_NAME}_core ${catkin_LIBRARIES})
target_compile_features(aw149_surround_obstacle_checker_node PUBLIC cxx_std_17)

install(TARGETS ${PROJECT_NAME}_core aw149_surround_obstacle_checker_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_surround_obstacle_checker_parameters
    parameter_adapter/test_surround_obstacle_checker_parameters.cpp)
  if(TARGET test_surround_obstacle_checker_parameters)
    target_link_libraries(test_surround_obstacle_checker_parameters ${catkin_LIBRARIES})
    target_compile_features(test_surround_obstacle_checker_parameters PUBLIC cxx_std_17)
  endif()
endif()
install(DIRECTORY include/ parameter_adapter/include/ DESTINATION include)

add_library(${PROJECT_NAME}_full_source src/node.cpp src/debug_marker.cpp)
target_compile_features(${PROJECT_NAME}_full_source PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_full_source ${catkin_LIBRARIES} ${PCL_LIBRARIES})
set_target_properties(${PROJECT_NAME}_full_source PROPERTIES LINK_FLAGS "-Wl,--no-undefined")
add_executable(surround_obstacle_checker_node src/full_main.cpp)
target_compile_features(surround_obstacle_checker_node PUBLIC cxx_std_17)
target_link_libraries(surround_obstacle_checker_node ${PROJECT_NAME}_full_source)
install(TARGETS ${PROJECT_NAME}_full_source surround_obstacle_checker_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(autoware_test_utils REQUIRED)
  foreach(test_name test_surround_obstacle_checker test_planning_factor test_native_contracts)
    catkin_add_gtest(${test_name} test/${test_name}.cpp)
    if(TARGET ${test_name})
      target_compile_features(${test_name} PUBLIC cxx_std_17)
      target_include_directories(${test_name} PRIVATE ${autoware_test_utils_INCLUDE_DIRS})
      target_link_libraries(${test_name} ${PROJECT_NAME}_full_source ${autoware_test_utils_LIBRARIES} ${catkin_LIBRARIES})
    endif()
  endforeach()
endif()
