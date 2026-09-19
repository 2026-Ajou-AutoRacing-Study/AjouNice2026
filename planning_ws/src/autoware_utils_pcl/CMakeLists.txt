cmake_minimum_required(VERSION 3.0.2)
project(autoware_utils_pcl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  autoware_utils_tf
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

get_target_property(EIGEN3_INCLUDE_DIRS Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
if(NOT EIGEN3_INCLUDE_DIRS)
  message(FATAL_ERROR "Eigen3::Eigen must export INTERFACE_INCLUDE_DIRECTORIES for public headers")
endif()
set(Eigen3_INCLUDE_DIRS "${EIGEN3_INCLUDE_DIRS}" CACHE INTERNAL "Eigen3 public include directories")

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS autoware_utils_tf pcl_conversions pcl_ros roscpp sensor_msgs
  DEPENDS Eigen3 PCL
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

if(CATKIN_ENABLE_TESTING)
  file(GLOB_RECURSE test_files test/*.cpp)
  catkin_add_gtest(test_${PROJECT_NAME} ${test_files})
  if(TARGET test_${PROJECT_NAME})
    target_link_libraries(test_${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} Eigen3::Eigen gtest_main)
  endif()
endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
