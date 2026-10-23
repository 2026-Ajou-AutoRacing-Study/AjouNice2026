cmake_minimum_required(VERSION 3.0.2)
project(autoware_path_smoother)

find_package(catkin REQUIRED COMPONENTS
  roscpp geometry_msgs nav_msgs std_msgs tf2 tf2_geometry_msgs
  autoware_internal_debug_msgs autoware_interpolation autoware_motion_utils
  autoware_osqp_interface autoware_planning_msgs autoware_utils_debug
  autoware_utils_geometry autoware_utils_rclcpp autoware_utils_system)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs std_msgs tf2 tf2_geometry_msgs
    autoware_internal_debug_msgs autoware_interpolation autoware_motion_utils
    autoware_osqp_interface autoware_planning_msgs autoware_utils_debug
    autoware_utils_geometry autoware_utils_rclcpp autoware_utils_system)

add_library(${PROJECT_NAME} SHARED
  src/elastic_band.cpp
  src/elastic_band_smoother.cpp
  src/replan_checker.cpp
  src/utils/trajectory_utils.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} BEFORE PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} Eigen3::Eigen)

add_executable(elastic_band_smoother_node src/main.cpp)
target_compile_features(elastic_band_smoother_node PUBLIC cxx_std_17)
target_link_libraries(elastic_band_smoother_node PRIVATE ${PROJECT_NAME} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_eb_history_reset test/eb_history_reset.test test/test_eb_history_reset.cpp)
  target_link_libraries(test_eb_history_reset ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

install(TARGETS ${PROJECT_NAME} elastic_band_smoother_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/autoware/path_smoother/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/autoware/path_smoother)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
