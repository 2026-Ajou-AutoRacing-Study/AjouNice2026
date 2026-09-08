cmake_minimum_required(VERSION 3.0.2)
project(autoware_lanelet2_utils)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(catkin REQUIRED COMPONENTS
  autoware_lanelet2_extension
  autoware_map_msgs
  autoware_planning_msgs
  autoware_utils_geometry
  autoware_utils_math
  geometry_msgs
  lanelet2_core
  lanelet2_io
  lanelet2_routing
  lanelet2_traffic_rules
  roscpp
  tf2
  tf2_geometry_msgs
)
find_package(lanelet2_projection REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES autoware_lanelet2_utils
  CATKIN_DEPENDS autoware_lanelet2_extension autoware_map_msgs autoware_planning_msgs autoware_utils_geometry autoware_utils_math geometry_msgs lanelet2_core lanelet2_io lanelet2_projection lanelet2_routing lanelet2_traffic_rules roscpp tf2 tf2_geometry_msgs
)
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(autoware_lanelet2_utils SHARED
  src/kind.cpp src/topology.cpp src/stop_line.cpp src/intersection.cpp src/conversion.cpp
  src/geometry.cpp src/hatched_road_markings.cpp src/lane_sequence.cpp src/nn_search.cpp
  src/map_handler.cpp src/route_manager.cpp
)
add_dependencies(autoware_lanelet2_utils ${catkin_EXPORTED_TARGETS})
target_link_libraries(autoware_lanelet2_utils ${catkin_LIBRARIES} ${lanelet2_projection_LIBRARIES})
add_executable(lanelet2_utils_map_roundtrip_oracle test/map_roundtrip_oracle.cpp)
target_link_libraries(lanelet2_utils_map_roundtrip_oracle autoware_lanelet2_utils ${catkin_LIBRARIES})
add_executable(lanelet2_utils_local_utm_map_oracle test/local_utm_map_oracle.cpp)
target_link_libraries(lanelet2_utils_local_utm_map_oracle autoware_lanelet2_utils ${catkin_LIBRARIES})
if(CATKIN_ENABLE_TESTING)
  find_package(yaml-cpp REQUIRED)
  catkin_add_gtest(test_autoware_lanelet2_utils_ported
    test/ros1_test_main.cpp
    test/conversion.cpp
  )
  if(TARGET test_autoware_lanelet2_utils_ported)
    target_include_directories(test_autoware_lanelet2_utils_ported PRIVATE include test)
    target_compile_definitions(test_autoware_lanelet2_utils_ported PRIVATE
      AUTOWARE_LANELET2_UTILS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(test_autoware_lanelet2_utils_ported
      autoware_lanelet2_utils ${catkin_LIBRARIES})
    target_compile_features(test_autoware_lanelet2_utils_ported PRIVATE cxx_std_17)
  endif()
  foreach(map_test geometry hatched_road_marking kind intersection lane_sequence map_handler nn_search route_manager stop_line topology)
    catkin_add_gtest(test_autoware_lanelet2_utils_${map_test}
      test/${map_test}.cpp)
    if(TARGET test_autoware_lanelet2_utils_${map_test})
      target_include_directories(test_autoware_lanelet2_utils_${map_test} PRIVATE include test)
      target_compile_definitions(test_autoware_lanelet2_utils_${map_test} PRIVATE
        AUTOWARE_LANELET2_UTILS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
      target_link_libraries(test_autoware_lanelet2_utils_${map_test}
        autoware_lanelet2_utils ${catkin_LIBRARIES} yaml-cpp)
      target_compile_features(test_autoware_lanelet2_utils_${map_test} PRIVATE cxx_std_17)
    endif()
  endforeach()
endif()
install(TARGETS autoware_lanelet2_utils
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(TARGETS lanelet2_utils_map_roundtrip_oracle
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS lanelet2_utils_local_utm_map_oracle
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/autoware/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/autoware
)
# Frozen upstream trajectory regressions resolve these fixtures through the
# lanelet2-utils package-share directory. Keep them as data only while making
# the ROS1 install underlay provide the same lookup surface.
install(DIRECTORY sample_map/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/sample_map
)
