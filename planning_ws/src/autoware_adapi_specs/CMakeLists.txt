cmake_minimum_required(VERSION 3.14)
project(autoware_adapi_specs)

find_package(catkin REQUIRED)


catkin_package(INCLUDE_DIRS include)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
