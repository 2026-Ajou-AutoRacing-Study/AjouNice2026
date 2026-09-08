cmake_minimum_required(VERSION 3.0.2)
project(tier4_datetime_rviz_plugin)

find_package(catkin REQUIRED COMPONENTS pluginlib roscpp rviz)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS pluginlib roscpp rviz)
include_directories(${catkin_INCLUDE_DIRS} include src)
add_library(${PROJECT_NAME} SHARED src/autoware_datetime_panel.hpp src/autoware_datetime_panel.cpp)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES plugins/plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/plugins)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
