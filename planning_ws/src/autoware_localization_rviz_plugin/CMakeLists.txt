cmake_minimum_required(VERSION 3.0.2)
project(autoware_localization_rviz_plugin)

find_package(catkin REQUIRED COMPONENTS
  autoware_internal_debug_msgs
  autoware_vehicle_info_utils
  geometry_msgs
  pluginlib
  rviz
  tf2_ros
)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(Eigen3 REQUIRED)

set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)
catkin_package()

add_library(${PROJECT_NAME} SHARED
  src/pose_history/pose_history_display.cpp
  src/pose_history_footprint/display.cpp
  src/pose_with_covariance_history/pose_with_covariance_history_display.cpp
  src/colored_pose_with_covariance_history/colored_pose_with_covariance_history_display.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_14)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Qt5::Widgets Eigen3::Eigen)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_plugin_discovery test/plugin_discovery.cpp)
  if(TARGET test_plugin_discovery)
    target_include_directories(test_plugin_discovery PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(test_plugin_discovery ${catkin_LIBRARIES} Qt5::Widgets gtest_main)
  endif()
endif()

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(FILES plugins/plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/plugins)
install(DIRECTORY icons images DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
