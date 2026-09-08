cmake_minimum_required(VERSION 3.0.2)
project(autoware_frenet_planner)

# The candidate preserves the frozen algorithms.  Only Noetic include and
# build bindings differ from the checked ROS 2 source.

find_package(catkin REQUIRED COMPONENTS
  autoware_sampler_common
  autoware_utils_math
  autoware_utils_geometry
  autoware_planning_msgs
  geometry_msgs
  tf2_geometry_msgs
)
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen3_LIBRARIES Eigen3::Eigen)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

add_library(${PROJECT_NAME} SHARED
  src/frenet_planner/frenet_planner.cpp
  src/frenet_planner/polynomials.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
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
  CATKIN_DEPENDS autoware_sampler_common autoware_utils_math autoware_utils_geometry autoware_planning_msgs geometry_msgs tf2_geometry_msgs
  DEPENDS Eigen3
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/autoware_frenet_planner/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/autoware_frenet_planner
)
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_frenet_semantics test/test_frenet_semantics.cpp)
  if(TARGET test_frenet_semantics)
    target_compile_features(test_frenet_semantics PRIVATE cxx_std_17)
    target_link_libraries(test_frenet_semantics ${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen gtest_main)
  endif()
endif()
