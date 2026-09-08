cmake_minimum_required(VERSION 3.0.2)
project(autoware_planning_topic_converter)

find_package(catkin REQUIRED COMPONENTS
  autoware_motion_utils
  autoware_planning_msgs
  autoware_utils_geometry
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES planning_topic_converter
  CATKIN_DEPENDS autoware_motion_utils autoware_planning_msgs autoware_utils_geometry roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_library(planning_topic_converter src/path_to_trajectory.cpp)
add_dependencies(planning_topic_converter ${catkin_EXPORTED_TARGETS})
target_link_libraries(planning_topic_converter ${catkin_LIBRARIES})

add_executable(path_to_trajectory_converter src/main.cpp)
target_link_libraries(path_to_trajectory_converter planning_topic_converter ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  add_executable(planning_topic_converter_runtime_oracle test/runtime_oracle.cpp)
  add_dependencies(planning_topic_converter_runtime_oracle ${catkin_EXPORTED_TARGETS})
  target_link_libraries(planning_topic_converter_runtime_oracle planning_topic_converter ${catkin_LIBRARIES})
endif()

install(TARGETS planning_topic_converter path_to_trajectory_converter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/autoware/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/autoware
)
