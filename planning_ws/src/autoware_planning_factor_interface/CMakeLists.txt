cmake_minimum_required(VERSION 3.0.2)
project(autoware_planning_factor_interface)

find_package(catkin REQUIRED COMPONENTS
  autoware_internal_planning_msgs
  autoware_motion_utils
  autoware_planning_msgs
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES autoware_planning_factor_interface
  CATKIN_DEPENDS autoware_internal_planning_msgs autoware_motion_utils autoware_planning_msgs roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(autoware_planning_factor_interface src/planing_factor_interface.cpp)
add_dependencies(autoware_planning_factor_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(autoware_planning_factor_interface ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  add_executable(planning_factor_interface_runtime_oracle test/runtime_oracle.cpp)
  add_dependencies(planning_factor_interface_runtime_oracle ${catkin_EXPORTED_TARGETS})
  target_link_libraries(planning_factor_interface_runtime_oracle
    autoware_planning_factor_interface ${catkin_LIBRARIES})
endif()

install(TARGETS autoware_planning_factor_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/autoware/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/autoware
)
