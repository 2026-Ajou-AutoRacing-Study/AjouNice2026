cmake_minimum_required(VERSION 3.0.2)
project(tier4_state_rviz_plugin)
find_package(catkin REQUIRED COMPONENTS
  autoware_adapi_specs
  autoware_adapi_v1_msgs autoware_component_interface_utils autoware_internal_planning_msgs
  autoware_motion_utils
  autoware_vehicle_msgs diagnostic_msgs geometry_msgs nav_msgs pluginlib roscpp rviz
  roslib tier4_external_api_msgs)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)
catkin_package()
file(GLOB STATE_PANEL_SOURCES src/*.cpp src/include/*.hpp)
add_library(${PROJECT_NAME} SHARED ${STATE_PANEL_SOURCES})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PRIVATE src/include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Qt5::Core Qt5::Widgets)
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY icons plugins images DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
