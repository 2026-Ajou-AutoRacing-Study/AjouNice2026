cmake_minimum_required(VERSION 3.10)
project(autoware_objects_of_interest_marker_interface)

add_compile_options(-std=c++17 -Wall -Wextra -Wpedantic -Werror)

find_package(catkin REQUIRED COMPONENTS
  autoware_perception_msgs
  autoware_utils_geometry
  autoware_utils_math
  autoware_utils_visualization
  geometry_msgs
  roscpp
  std_msgs
  visualization_msgs
)
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS autoware_perception_msgs autoware_utils_geometry autoware_utils_math autoware_utils_visualization geometry_msgs roscpp std_msgs visualization_msgs
  DEPENDS Eigen3
)

include_directories(include/${PROJECT_NAME} ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/coloring.cpp
  src/objects_of_interest_marker_interface.cpp
  src/marker_utils.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME}
    test/main.cpp
    test/marker_interface.cpp
  )
  if(TARGET test_${PROJECT_NAME})
    target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/autoware/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/autoware)
