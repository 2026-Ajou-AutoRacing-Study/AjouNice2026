cmake_minimum_required(VERSION 3.0.2)
project(autoware_map_loader)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  autoware_map_msgs
  autoware_geography_utils
  autoware_lanelet2_extension
  autoware_lanelet2_utils
  sensor_msgs
  pcl_conversions
  autoware_utils_rclcpp
)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lanelet2_map_loader pointcloud_map_loader autoware_map_loader_full_source
  CATKIN_DEPENDS roscpp autoware_map_msgs autoware_geography_utils autoware_lanelet2_extension autoware_lanelet2_utils sensor_msgs pcl_conversions autoware_utils_rclcpp
  DEPENDS PCL yaml-cpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

# Some older staged geography-utils exports retain an earlier lanelet extension
# config path. Resolve the canonical public header root from the active overlay.
find_path(AUTOWARE_LANELET2_EXTENSION_PUBLIC_INCLUDE
  NAMES autoware_lanelet2_extension/io/autoware_osm_parser.hpp
  PATHS ${CMAKE_PREFIX_PATH}
  PATH_SUFFIXES include
  NO_DEFAULT_PATH
)
if(NOT AUTOWARE_LANELET2_EXTENSION_PUBLIC_INCLUDE)
  message(FATAL_ERROR "canonical autoware_lanelet2_extension headers are unavailable")
endif()

add_library(lanelet2_map_loader src/lanelet2_map_loader.cpp)
target_include_directories(lanelet2_map_loader PRIVATE ${AUTOWARE_LANELET2_EXTENSION_PUBLIC_INCLUDE})
target_link_libraries(lanelet2_map_loader ${catkin_LIBRARIES})

add_executable(autoware_lanelet2_map_loader src/lanelet2_map_loader_node.cpp)
target_include_directories(autoware_lanelet2_map_loader PRIVATE ${AUTOWARE_LANELET2_EXTENSION_PUBLIC_INCLUDE})
target_link_libraries(autoware_lanelet2_map_loader lanelet2_map_loader ${catkin_LIBRARIES})

add_library(pointcloud_map_loader src/pointcloud_map_loader.cpp)
target_include_directories(pointcloud_map_loader SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pointcloud_map_loader ${catkin_LIBRARIES} ${PCL_LIBRARIES} yaml-cpp)
add_executable(autoware_pointcloud_map_loader src/pointcloud_map_loader_node.cpp)
target_link_libraries(autoware_pointcloud_map_loader pointcloud_map_loader ${catkin_LIBRARIES} ${PCL_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_lanelet2_map_loader test/test_lanelet2_map_loader.cpp)
  if(TARGET test_lanelet2_map_loader)
    target_compile_definitions(test_lanelet2_map_loader PRIVATE
      AUTOWARE_MAP_LOADER_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
    target_link_libraries(test_lanelet2_map_loader lanelet2_map_loader ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_pointcloud_map_loader test/test_pointcloud_map_loader.cpp)
  if(TARGET test_pointcloud_map_loader)
    target_link_libraries(test_pointcloud_map_loader pointcloud_map_loader ${catkin_LIBRARIES} ${PCL_LIBRARIES} yaml-cpp)
  endif()
endif()

install(TARGETS lanelet2_map_loader autoware_lanelet2_map_loader pointcloud_map_loader autoware_pointcloud_map_loader
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch test/data DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

find_package(autoware_utils_rclcpp REQUIRED CONFIG)
find_package(fmt REQUIRED)
add_library(autoware_map_loader_full_source SHARED
  src/full_source/lanelet2_map_loader/lanelet2_map_loader_node.cpp
  src/full_source/pointcloud_map_loader/differential_map_loader_module.cpp
  src/full_source/pointcloud_map_loader/partial_map_loader_module.cpp
  src/full_source/pointcloud_map_loader/pointcloud_map_loader_module.cpp
  src/full_source/pointcloud_map_loader/pointcloud_map_loader_node.cpp
  src/full_source/pointcloud_map_loader/selected_map_loader_module.cpp
  src/full_source/pointcloud_map_loader/utils.cpp)
target_include_directories(autoware_map_loader_full_source BEFORE PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src/full_source
  ${autoware_utils_rclcpp_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(autoware_map_loader_full_source ${catkin_LIBRARIES} ${PCL_LIBRARIES} yaml-cpp fmt::fmt)
set_target_properties(autoware_map_loader_full_source PROPERTIES LINK_FLAGS "-Wl,--no-undefined")
target_link_libraries(pointcloud_map_loader autoware_map_loader_full_source)
add_executable(autoware_pointcloud_map_loader_full src/full_pointcloud_main.cpp)
target_link_libraries(autoware_pointcloud_map_loader_full autoware_map_loader_full_source)
add_executable(autoware_lanelet2_map_loader_full src/full_lanelet2_main.cpp)
target_link_libraries(autoware_lanelet2_map_loader_full autoware_map_loader_full_source)
install(TARGETS autoware_map_loader_full_source autoware_pointcloud_map_loader_full autoware_lanelet2_map_loader_full
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/autoware/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/autoware)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gmock(full_test_cylinder_box_overlap test/full_original/test_cylinder_box_overlap.cpp)
  if(TARGET full_test_cylinder_box_overlap)
    target_link_libraries(full_test_cylinder_box_overlap autoware_map_loader_full_source gmock_main)
  endif()
  catkin_add_gmock(full_test_differential_map_loader_module test/full_original/test_differential_map_loader_module.cpp)
  if(TARGET full_test_differential_map_loader_module)
    target_link_libraries(full_test_differential_map_loader_module autoware_map_loader_full_source)
  endif()
  catkin_add_gmock(full_test_load_pcd_metadata test/full_original/test_load_pcd_metadata.cpp)
  if(TARGET full_test_load_pcd_metadata)
    target_link_libraries(full_test_load_pcd_metadata autoware_map_loader_full_source)
  endif()
  catkin_add_gmock(full_test_partial_map_loader_module test/full_original/test_partial_map_loader_module.cpp)
  if(TARGET full_test_partial_map_loader_module)
    target_link_libraries(full_test_partial_map_loader_module autoware_map_loader_full_source)
  endif()
  catkin_add_gmock(full_test_pointcloud_map_loader_module test/full_original/test_pointcloud_map_loader_module.cpp)
  if(TARGET full_test_pointcloud_map_loader_module)
    target_link_libraries(full_test_pointcloud_map_loader_module autoware_map_loader_full_source)
  endif()
  catkin_add_gmock(full_test_replace_with_absolute_path test/full_original/test_replace_with_absolute_path.cpp)
  if(TARGET full_test_replace_with_absolute_path)
    target_link_libraries(full_test_replace_with_absolute_path autoware_map_loader_full_source)
  endif()
endif()
