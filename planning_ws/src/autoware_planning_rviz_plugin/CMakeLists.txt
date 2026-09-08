cmake_minimum_required(VERSION 3.0.2)
project(autoware_planning_rviz_plugin)

find_package(catkin REQUIRED COMPONENTS
  autoware_internal_planning_msgs
  autoware_planning_msgs
  autoware_utils_geometry
  autoware_utils_math
  autoware_vehicle_info_utils
  geometry_msgs
  pluginlib
  roscpp
  rviz
  rviz_2d_overlay_plugins
  tf2_geometry_msgs
  tf2_ros
)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(Eigen3 REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)
catkin_package(INCLUDE_DIRS include)

file(GLOB_RECURSE PLANNING_RVIZ_HEADERS include/*.hpp src/tools/*.hpp)
file(GLOB_RECURSE PLANNING_RVIZ_SOURCES src/*.cpp)
add_library(${PROJECT_NAME} SHARED ${PLANNING_RVIZ_HEADERS} ${PLANNING_RVIZ_SOURCES})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Qt5::Core Qt5::Gui Qt5::Widgets Eigen3::Eigen)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES plugins/plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/plugins)
install(DIRECTORY icons DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} OPTIONAL)
