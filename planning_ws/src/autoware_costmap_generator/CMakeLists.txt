cmake_minimum_required(VERSION 3.13)
project(autoware_costmap_generator)

find_package(catkin REQUIRED COMPONENTS autoware_perception_msgs autoware_map_msgs autoware_internal_planning_msgs autoware_internal_debug_msgs autoware_utils autoware_lanelet2_utils autoware_lanelet2_extension geometry_msgs nav_msgs sensor_msgs grid_map_core grid_map_ros grid_map_msgs pcl_conversions pcl_ros roscpp tf2_geometry_msgs tf2_ros tf2_eigen)
find_package(PCL REQUIRED COMPONENTS common io)
include("${CMAKE_CURRENT_LIST_DIR}/../cmake/AW149Providers.cmake")
aw149_provider(GRID_MAP_UTILS autoware_grid_map_utils autoware_grid_map_utils)
set(GRID_MAP_UTILS_INCLUDE_DIR "" CACHE PATH "installed autoware_grid_map_utils include directory")
set(GRID_MAP_UTILS_LIBRARY "" CACHE FILEPATH "installed autoware_grid_map_utils library")
foreach(path IN ITEMS GRID_MAP_UTILS_INCLUDE_DIR GRID_MAP_UTILS_LIBRARY)
  if(NOT EXISTS "${${path}}")
    message(FATAL_ERROR "Set ${path} to the staged ROS1 provider")
  endif()
endforeach()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS autoware_perception_msgs autoware_map_msgs autoware_internal_planning_msgs autoware_internal_debug_msgs autoware_utils autoware_lanelet2_utils autoware_lanelet2_extension geometry_msgs nav_msgs sensor_msgs grid_map_core grid_map_ros grid_map_msgs pcl_conversions pcl_ros roscpp tf2_geometry_msgs tf2_ros tf2_eigen
  DEPENDS PCL
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${GRID_MAP_UTILS_INCLUDE_DIR})

add_library(${PROJECT_NAME} SHARED src/points_to_costmap.cpp src/object_map_utils.cpp src/objects_to_costmap.cpp src/costmap_core.cpp src/costmap_generator.cpp)
target_link_libraries(${PROJECT_NAME} ${GRID_MAP_UTILS_LIBRARY} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_options(${PROJECT_NAME} PRIVATE -Wl,--no-undefined)
add_executable(costmap_generator_node src/main.cpp)
target_link_libraries(costmap_generator_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(costmap_generator_publish_core src/costmap_generator_publish_core.cpp)
target_link_libraries(costmap_generator_publish_core ${PROJECT_NAME} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(autoware_test_utils REQUIRED CONFIG)
  catkin_add_gtest(test_original_costmap
    test/original_test_costmap_generator.cpp test/original_test_points_to_costmap.cpp
    test/original_test_objects_to_costmap.cpp test/original_test_object_map_utils.cpp test/ros1_main.cpp)
  if(TARGET test_original_costmap)
    target_link_libraries(test_original_costmap ${PROJECT_NAME} ${autoware_test_utils_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  endif()
  catkin_add_gtest(test_points_to_costmap test/test_points_to_costmap.cpp)
  if(TARGET test_points_to_costmap)
    target_link_libraries(test_points_to_costmap ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  endif()
endif()

install(TARGETS ${PROJECT_NAME} costmap_generator_node LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION include)
install(FILES package.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
