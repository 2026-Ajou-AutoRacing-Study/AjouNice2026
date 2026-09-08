cmake_minimum_required(VERSION 3.0.2)
project(tl_expected)

find_package(catkin REQUIRED)

catkin_package(INCLUDE_DIRS include)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
