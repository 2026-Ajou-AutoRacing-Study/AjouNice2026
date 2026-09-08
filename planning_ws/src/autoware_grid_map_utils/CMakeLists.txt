cmake_minimum_required(VERSION 3.0.2)
project(autoware_grid_map_utils)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS grid_map_core)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS grid_map_core
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED src/polygon_iterator.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
