cmake_minimum_required(VERSION 3.0.2)
project(autoware_geography_utils)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  autoware_lanelet2_extension
  autoware_map_msgs
  geographic_msgs
  geometry_msgs
  lanelet2_io
)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h)
find_library(GeographicLib_LIBRARY NAMES Geographic REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${GeographicLib_INCLUDE_DIR}
  LIBRARIES autoware_geography_utils
  CATKIN_DEPENDS autoware_lanelet2_extension autoware_map_msgs geographic_msgs
    geometry_msgs lanelet2_io
)

add_library(autoware_geography_utils SHARED
  src/height.cpp
  src/projection.cpp
  src/lanelet2_projector.cpp
)
target_include_directories(autoware_geography_utils PUBLIC
  include ${catkin_INCLUDE_DIRS} ${GeographicLib_INCLUDE_DIR}
)
target_link_libraries(autoware_geography_utils PUBLIC ${catkin_LIBRARIES} ${GeographicLib_LIBRARY})
target_compile_features(autoware_geography_utils PUBLIC cxx_std_17)

install(TARGETS autoware_geography_utils
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/autoware/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/autoware
)
