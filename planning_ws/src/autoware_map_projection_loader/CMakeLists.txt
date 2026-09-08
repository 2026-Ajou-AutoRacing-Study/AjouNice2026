cmake_minimum_required(VERSION 3.0.2)
project(autoware_map_projection_loader)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  autoware_map_msgs
  geographic_msgs
  autoware_lanelet2_extension
)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp autoware_map_msgs geographic_msgs autoware_lanelet2_extension
  DEPENDS yaml-cpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/map_projection_loader.cpp
  src/load_info_from_lanelet2_map.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

add_executable(${PROJECT_NAME}_node src/map_projection_loader_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_map_projection_loader test/test_map_projection_loader.cpp)
  if(TARGET test_map_projection_loader)
    target_link_libraries(test_map_projection_loader ${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)
  endif()
endif()

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch config test/data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
