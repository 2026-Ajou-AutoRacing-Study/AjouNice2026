cmake_minimum_required(VERSION 3.0.2)
project(rviz_2d_overlay_plugins)

find_package(catkin REQUIRED COMPONENTS pluginlib roscpp rviz rviz_2d_overlay_msgs std_msgs)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME})

add_library(${PROJECT_NAME} SHARED
  include/rviz_2d_overlay_plugins/overlay_text_display.hpp
  include/rviz_2d_overlay_plugins/pie_chart_display.h
  src/overlay_text_display.cpp
  src/overlay_utils.cpp
  src/pie_chart_display.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Qt5::Core Qt5::Gui Qt5::Widgets)

add_executable(string_to_overlay_text src/string_to_overlay_text.cpp)
target_compile_features(string_to_overlay_text PUBLIC cxx_std_17)
target_include_directories(string_to_overlay_text PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(string_to_overlay_text ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} string_to_overlay_text
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(FILES plugins_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
