cmake_minimum_required(VERSION 3.0.2)
project(trajectory_follower)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  autoware_planning_msgs
  morai_msgs
  nav_msgs
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS autoware_planning_msgs morai_msgs nav_msgs roscpp std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(purepursuit src/purepursuit.cpp)
add_executable(longi_pi src/longi_pi.cpp)
add_dependencies(purepursuit ${catkin_EXPORTED_TARGETS})
add_dependencies(longi_pi ${catkin_EXPORTED_TARGETS})
target_link_libraries(purepursuit ${catkin_LIBRARIES})
target_link_libraries(longi_pi ${catkin_LIBRARIES})

install(TARGETS purepursuit longi_pi
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
