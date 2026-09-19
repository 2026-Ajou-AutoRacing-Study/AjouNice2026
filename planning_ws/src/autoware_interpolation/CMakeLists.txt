cmake_minimum_required(VERSION 3.0.2)
project(autoware_interpolation)

find_package(catkin REQUIRED COMPONENTS
  autoware_internal_planning_msgs
  autoware_planning_msgs
  autoware_utils_geometry
  geometry_msgs
  tf2
  tf2_geometry_msgs
)
find_package(Eigen3 REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

add_library(${PROJECT_NAME} SHARED
  src/linear_interpolation.cpp
  src/spline_interpolation.cpp
  src/spline_interpolation_points_2d.cpp
  src/spherical_linear_interpolation.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CATKIN_GLOBAL_INCLUDE_DESTINATION}>
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS autoware_internal_planning_msgs autoware_planning_msgs autoware_utils_geometry geometry_msgs tf2 tf2_geometry_msgs
  DEPENDS EIGEN3
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  file(GLOB_RECURSE test_files test/*.cpp)
  catkin_add_gtest(test_${PROJECT_NAME} ${test_files})
  if(TARGET test_${PROJECT_NAME})
    target_compile_features(test_${PROJECT_NAME} PRIVATE cxx_std_17)
    target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen)
  endif()
endif()
