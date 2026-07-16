cmake_minimum_required(VERSION 3.0.2)
project(purepursuit_pi)

find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs geometry_msgs nav_msgs visualization_msgs tf2_ros tf2_geometry_msgs
  autoware_planning_msgs morai_msgs
)
find_package(OpenSSL REQUIRED)

catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS
  roscpp std_msgs geometry_msgs nav_msgs visualization_msgs tf2_ros tf2_geometry_msgs
  autoware_planning_msgs morai_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(pp_pi_controller
  src/controller_node.cpp
  src/pure_pursuit_lateral.cpp
  src/pi_longitudinal.cpp
  src/ego_state_provider.cpp
  src/command_guard.cpp
)
target_link_libraries(pp_pi_controller ${catkin_LIBRARIES} OpenSSL::Crypto)
add_dependencies(pp_pi_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS pp_pi_controller RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
