cmake_minimum_required(VERSION 3.14)
project(autoware_utils_geometry)
find_package(catkin REQUIRED COMPONENTS roscpp tf2 tf2_eigen tf2_geometry_msgs autoware_internal_planning_msgs autoware_planning_msgs autoware_perception_msgs autoware_utils_math)
find_package(Boost REQUIRED COMPONENTS system)
# Common-only projection. Not a package-complete CMakeLists.txt.
# Exact autoware_package compile-policy effects applicable to this common root.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24.0")
    option(CMAKE_COMPILE_WARNING_AS_ERROR "Treat compiler warnings as errors." ON)
    mark_as_advanced(CMAKE_COMPILE_WARNING_AS_ERROR)
  else()
    add_compile_options(-Werror)
  endif()
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-gnu-anonymous-struct -Wno-nested-anon-types)
endif()
add_compile_definitions(BOOST_ALLOW_DEPRECATED_HEADERS)

# Source-bound ament_auto_add_library projection only. ABI/TF/link closure is excluded.
add_library(${PROJECT_NAME} SHARED
  "src/geometry/alt_geometry.cpp"
  "src/geometry/boost_polygon_utils.cpp"
  "src/geometry/ear_clipping.cpp"
  "src/geometry/geometry.cpp"
  "src/geometry/gjk_2d.cpp"
  "src/geometry/pose_deviation.cpp"
  "src/geometry/random_concave_polygon.cpp"
  "src/geometry/random_convex_polygon.cpp"
  "src/geometry/sat_2d.cpp"
  "src/msg/operation.cpp"
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CATKIN_PACKAGE_INCLUDE_DESTINATION}>
  ${catkin_INCLUDE_DIRS}
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS autoware_internal_planning_msgs autoware_planning_msgs autoware_perception_msgs autoware_utils_math
  
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  file(GLOB_RECURSE test_files test/*.cpp)
  catkin_add_gtest(test_${PROJECT_NAME} ${test_files})
  if(TARGET test_${PROJECT_NAME})
    target_compile_features(test_${PROJECT_NAME} PRIVATE cxx_std_17)
  endif()
endif()

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Boost::system)
if(TARGET test_${PROJECT_NAME})
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} ${catkin_LIBRARIES} Boost::system)
endif()
