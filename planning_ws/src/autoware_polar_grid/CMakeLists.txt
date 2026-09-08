cmake_minimum_required(VERSION 3.14)
project(autoware_polar_grid)

find_package(catkin REQUIRED COMPONENTS
  rviz
  pluginlib
)

find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)

add_library(${PROJECT_NAME} SHARED
  src/polar_grid_display.cpp
  src/polar_grid_display.hpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_package(
  CATKIN_DEPENDS rviz pluginlib
  DEPENDS Qt5
)

install(DIRECTORY icons plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
