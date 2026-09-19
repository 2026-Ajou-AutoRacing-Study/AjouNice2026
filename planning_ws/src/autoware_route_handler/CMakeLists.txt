cmake_minimum_required(VERSION 3.0.2)
project(autoware_route_handler)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  autoware_internal_planning_msgs
  autoware_lanelet2_extension
  autoware_lanelet2_utils
  autoware_map_msgs
  autoware_planning_msgs
  autoware_trajectory
  autoware_utils_geometry
  autoware_utils_math
  geometry_msgs
  lanelet2_core
  lanelet2_routing
  lanelet2_traffic_rules
  roscpp
  std_msgs
  tf2
  unique_identifier_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES autoware_route_handler
  CATKIN_DEPENDS autoware_internal_planning_msgs autoware_lanelet2_extension
    autoware_lanelet2_utils autoware_map_msgs autoware_planning_msgs
    autoware_trajectory autoware_utils_geometry autoware_utils_math geometry_msgs
    lanelet2_core lanelet2_routing lanelet2_traffic_rules roscpp std_msgs tf2
    unique_identifier_msgs
)

add_library(autoware_route_handler SHARED src/route_handler.cpp)
target_include_directories(autoware_route_handler PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(autoware_route_handler PUBLIC ${catkin_LIBRARIES})
target_compile_features(autoware_route_handler PUBLIC cxx_std_17)

add_executable(route_handler_runtime_oracle test/route_handler_runtime_oracle.cpp)
target_link_libraries(route_handler_runtime_oracle PRIVATE autoware_route_handler ${catkin_LIBRARIES})
target_compile_features(route_handler_runtime_oracle PRIVATE cxx_std_17)

install(TARGETS autoware_route_handler
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS route_handler_runtime_oracle
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/autoware/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/autoware
)
