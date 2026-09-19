cmake_minimum_required(VERSION 3.0.2)
project(autoware_agnocast_wrapper)

# This ROS 1 package deliberately has one build profile.  Do not turn an
# enabled Agnocast request into an ordinary ROS transport build.
set(_aw_agnocast_requested "$ENV{ENABLE_AGNOCAST}")
if(NOT "${_aw_agnocast_requested}" STREQUAL "" AND
   NOT "${_aw_agnocast_requested}" STREQUAL "0")
  message(FATAL_ERROR
    "autoware_agnocast_wrapper ROS1 supports only ENABLE_AGNOCAST=0 (or unset); "
    "ENABLE_AGNOCAST=${_aw_agnocast_requested} requires Agnocast IPC, which is not provided by this ROS1 port.")
endif()

find_package(catkin REQUIRED COMPONENTS
  message_filters
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_filters roscpp
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

