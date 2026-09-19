cmake_minimum_required(VERSION 3.0.2)
project(autoware_sampler_common)

# The original implementation is included in the distributable ROS1 source.
set(SAMPLER_COMMON_ROS2_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/frozen_source")
if(NOT EXISTS "${SAMPLER_COMMON_ROS2_SOURCE}/include/autoware_sampler_common/structures.hpp")
  message(FATAL_ERROR "SAMPLER_COMMON_ROS2_SOURCE does not name the frozen autoware_sampler_common source")
endif()

find_package(catkin REQUIRED COMPONENTS
  autoware_interpolation
  autoware_utils_geometry
  geometry_msgs
)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

add_library(${PROJECT_NAME} SHARED
  "${SAMPLER_COMMON_ROS2_SOURCE}/src/sampler_common/constraints/footprint.cpp"
  "${SAMPLER_COMMON_ROS2_SOURCE}/src/sampler_common/constraints/hard_constraint.cpp"
  "${SAMPLER_COMMON_ROS2_SOURCE}/src/sampler_common/constraints/soft_constraint.cpp"
  "${SAMPLER_COMMON_ROS2_SOURCE}/src/sampler_common/transform/spline_transform.cpp"
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${SAMPLER_COMMON_ROS2_SOURCE}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CATKIN_GLOBAL_INCLUDE_DESTINATION}>
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} Eigen3::Eigen)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS autoware_interpolation autoware_utils_geometry geometry_msgs
  DEPENDS Boost EIGEN3
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY "${SAMPLER_COMMON_ROS2_SOURCE}/include/autoware_sampler_common/"
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/autoware_sampler_common
)
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_sampler_common
    test/test_spline_endpoint.cpp
    "${SAMPLER_COMMON_ROS2_SOURCE}/test/test_transform.cpp"
    "${SAMPLER_COMMON_ROS2_SOURCE}/test/test_structures.cpp"
  )
  if(TARGET test_sampler_common)
    target_compile_features(test_sampler_common PRIVATE cxx_std_17)
    # ament_add_gtest supplies a test main in ROS 2; catkin_add_gtest does
    # not, so retain the upstream test sources and supply its ROS 1 analogue.
    target_link_libraries(
      test_sampler_common
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
      Eigen3::Eigen
      gtest_main
    )
  endif()
endif()
