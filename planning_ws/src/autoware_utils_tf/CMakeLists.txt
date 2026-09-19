cmake_minimum_required(VERSION 3.0.2)
project(autoware_utils_tf)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  autoware_utils_geometry
  geometry_msgs
  roscpp
  tf2_ros
)
find_package(Eigen3 REQUIRED)
get_target_property(EIGEN3_INCLUDE_DIRS Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
if(NOT EIGEN3_INCLUDE_DIRS)
  message(FATAL_ERROR "Eigen3::Eigen must export INTERFACE_INCLUDE_DIRECTORIES for public headers")
endif()

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS autoware_utils_geometry geometry_msgs roscpp tf2_ros
  DEPENDS Eigen3
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_api test/test_api.cpp)
  if(TARGET test_api)
    target_include_directories(test_api PRIVATE ${EIGEN3_INCLUDE_DIRS})
    target_link_libraries(test_api ${catkin_LIBRARIES} Eigen3::Eigen gtest_main)
  endif()
endif()

add_executable(tf_runtime_oracle test/tf_runtime_oracle.cpp)
target_include_directories(tf_runtime_oracle PRIVATE ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(tf_runtime_oracle ${catkin_LIBRARIES} Eigen3::Eigen)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS tf_runtime_oracle
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
