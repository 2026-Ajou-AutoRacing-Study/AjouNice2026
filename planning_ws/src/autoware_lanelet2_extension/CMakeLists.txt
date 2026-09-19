cmake_minimum_required(VERSION 3.0.2)
project(autoware_lanelet2_extension)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  autoware_map_msgs
  autoware_planning_msgs
  geometry_msgs
  lanelet2_core
  lanelet2_io
  lanelet2_maps
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules
  lanelet2_validation
  roscpp
  tf2
  tf2_geometry_msgs
  visualization_msgs
)
find_package(GeographicLib REQUIRED)
find_package(PugiXML REQUIRED)
# The Debian PugiXML config exports upper-case variables, while catkin's
# DEPENDS export convention uses the package spelling below.
set(PugiXML_INCLUDE_DIRS ${PUGIXML_INCLUDE_DIRS})
set(PugiXML_LIBRARIES ${PUGIXML_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES autoware_lanelet2_extension_lib
  CATKIN_DEPENDS autoware_map_msgs autoware_planning_msgs geometry_msgs lanelet2_core lanelet2_io lanelet2_maps lanelet2_projection lanelet2_routing lanelet2_traffic_rules lanelet2_validation roscpp tf2 tf2_geometry_msgs visualization_msgs
  DEPENDS GeographicLib PugiXML
)

include_directories(include ${catkin_INCLUDE_DIRS} ${GeographicLib_INCLUDE_DIRS} ${PUGIXML_INCLUDE_DIRS})
add_library(autoware_lanelet2_extension_lib SHARED
  lib/autoware_traffic_rules.cpp
  lib/autoware_osm_parser.cpp
  lib/autoware_traffic_light.cpp
  lib/crosswalk.cpp
  lib/detection_area.cpp
  lib/landmark.cpp
  lib/no_parking_area.cpp
  lib/no_stopping_area.cpp
  lib/bus_stop_area.cpp
  lib/message_conversion.cpp
  lib/mgrs_projector.cpp
  lib/query.cpp
  lib/roundabout.cpp
  lib/road_marking.cpp
  lib/speed_bump.cpp
  lib/transverse_mercator_projector.cpp
  lib/utilities.cpp
  lib/deprecated.cpp
  lib/virtual_traffic_light.cpp
  lib/visualization.cpp
  lib/route_checker.cpp
)
add_dependencies(autoware_lanelet2_extension_lib ${catkin_EXPORTED_TARGETS})
target_compile_options(autoware_lanelet2_extension_lib PRIVATE -Wno-error=maybe-uninitialized)
target_link_libraries(autoware_lanelet2_extension_lib ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES} ${PUGIXML_LIBRARIES})

add_executable(lanelet2_extension_oracle test/lanelet2_extension_oracle.cpp)
target_link_libraries(lanelet2_extension_oracle autoware_lanelet2_extension_lib ${catkin_LIBRARIES})
if(CATKIN_ENABLE_TESTING)
  foreach(extension_test normalize_radian projector query regulatory_elements route_checker utilities visualization)
    catkin_add_gtest(test_autoware_lanelet2_extension_${extension_test}
      test/src/test_${extension_test}.cpp)
    if(TARGET test_autoware_lanelet2_extension_${extension_test})
      target_link_libraries(test_autoware_lanelet2_extension_${extension_test}
        autoware_lanelet2_extension_lib ${catkin_LIBRARIES})
      target_compile_features(test_autoware_lanelet2_extension_${extension_test} PRIVATE cxx_std_17)
    endif()
  endforeach()
endif()

install(TARGETS autoware_lanelet2_extension_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(TARGETS lanelet2_extension_oracle
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
