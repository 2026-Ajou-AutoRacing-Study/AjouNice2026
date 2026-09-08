cmake_minimum_required(VERSION 3.0.2)
project(tier4_vehicle_rviz_plugin)

find_package(catkin REQUIRED COMPONENTS
  autoware_utils
  autoware_vehicle_msgs
  geometry_msgs
  pluginlib
  roscpp
  rviz
)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)

catkin_package()

set(TIER4_VEHICLE_RVIZ_SOURCES
  src/tools/turn_signal.cpp
  src/tools/console_meter.cpp
  src/tools/steering_angle.cpp
  src/tools/jsk_overlay_utils.cpp
  src/tools/velocity_history.cpp
  src/tools/acceleration_meter.cpp
)
add_library(${PROJECT_NAME} SHARED ${TIER4_VEHICLE_RVIZ_SOURCES})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PRIVATE src ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Qt5::Core Qt5::Gui Qt5::Widgets)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY plugins icons images DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
