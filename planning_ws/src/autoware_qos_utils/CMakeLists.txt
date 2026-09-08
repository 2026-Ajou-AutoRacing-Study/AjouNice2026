cmake_minimum_required(VERSION 3.0.2)
project(autoware_qos_utils)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

install(DIRECTORY include/autoware/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/autoware
)
