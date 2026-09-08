cmake_minimum_required(VERSION 3.10)
project(autoware_perception_rviz_plugin)
find_package(catkin REQUIRED COMPONENTS roscpp rviz pluginlib tf2_geometry_msgs tf2_ros
  autoware_lanelet2_extension autoware_map_msgs autoware_perception_msgs
  unique_identifier_msgs visualization_msgs)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_io REQUIRED)
get_filename_component(RVIZ_PREFIX "${rviz_DIR}/../../.." ABSOLUTE)
find_library(RVIZ_DEFAULT_PLUGIN_LIBRARY NAMES rviz_default_plugin
  HINTS "${RVIZ_PREFIX}/lib" NO_DEFAULT_PATH REQUIRED)
set(CMAKE_AUTOMOC ON)
set(CMAKE_CXX_STANDARD 17)
catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rviz pluginlib tf2_geometry_msgs tf2_ros
  autoware_lanelet2_extension autoware_map_msgs autoware_perception_msgs
  unique_identifier_msgs visualization_msgs)
add_library(${PROJECT_NAME} SHARED
  src/common/color_alpha_property.cpp
  src/object_detection/object_polygon_detail.cpp
  src/object_detection/detected_objects_display.cpp
  src/object_detection/tracked_objects_display.cpp
  src/object_detection/predicted_objects_display.cpp
  src/traffic_light/traffic_light_display.cpp
  include/autoware_perception_rviz_plugin/object_detection/detected_objects_display.hpp
  include/autoware_perception_rviz_plugin/object_detection/tracked_objects_display.hpp
  include/autoware_perception_rviz_plugin/object_detection/predicted_objects_display.hpp
  include/autoware_perception_rviz_plugin/traffic_light/traffic_light_display.hpp)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${lanelet2_core_INCLUDE_DIRS} ${lanelet2_io_INCLUDE_DIRS})
target_compile_definitions(${PROJECT_NAME} PRIVATE OBJECT_DETECTION_PLUGINS_BUILDING_LIBRARY)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Qt5::Widgets
  ${RVIZ_DEFAULT_PLUGIN_LIBRARY} ${Boost_LIBRARIES} ${lanelet2_core_LIBRARIES} ${lanelet2_io_LIBRARIES})
target_link_options(${PROJECT_NAME} PRIVATE -Wl,--no-undefined)
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(DIRECTORY icons meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES plugins_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_original_perception_geometry test/test_original_geometry.cpp)
  if(TARGET test_original_perception_geometry)
    target_link_libraries(test_original_perception_geometry ${PROJECT_NAME})
    add_custom_target(build_original_perception_tests ALL DEPENDS test_original_perception_geometry)
  endif()
endif()
