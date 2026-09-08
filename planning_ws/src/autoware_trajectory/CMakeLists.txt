cmake_minimum_required(VERSION 3.0.2)
project(autoware_trajectory)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  autoware_internal_planning_msgs
  autoware_lanelet2_utils
  autoware_planning_msgs
  autoware_utils_geometry
  geometry_msgs
  lanelet2_core
  roscpp
  tf2
  tf2_geometry_msgs
  tl_expected
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES autoware_trajectory
  CATKIN_DEPENDS autoware_internal_planning_msgs autoware_lanelet2_utils
    autoware_planning_msgs autoware_utils_geometry geometry_msgs lanelet2_core
    roscpp tf2 tf2_geometry_msgs tl_expected
  DEPENDS Eigen3 fmt
)

add_library(autoware_trajectory SHARED
  src/threshold.cpp
  src/point.cpp
  src/pose.cpp
  src/path_point.cpp
  src/path_point_with_lane_id.cpp
  src/trajectory_point.cpp
  src/detail/logging.cpp
  src/detail/time_distance_mapping.cpp
  src/detail/types.cpp
  src/detail/util.cpp
  src/interpolator/akima_spline.cpp
  src/interpolator/cubic_spline.cpp
  src/interpolator/lane_ids_interpolator.cpp
  src/interpolator/linear.cpp
  src/interpolator/pchip.cpp
  src/interpolator/result.cpp
  src/interpolator/spherical_linear.cpp
  src/temporal_trajectory.cpp
  src/utils/closest.cpp
  src/utils/crossed.cpp
  src/utils/crop.cpp
  src/utils/find_if.cpp
  src/utils/find_intervals.cpp
  src/utils/find_nearest.cpp
  src/utils/footprint.cpp
  src/utils/align_orientation.cpp
  src/utils/pretty_build.cpp
  src/utils/reference_path.cpp
  src/utils/set_stopline.cpp
  src/utils/set_time_offset.cpp
  src/utils/shift.cpp
  src/utils/velocity.cpp
)
target_include_directories(autoware_trajectory PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(autoware_trajectory PUBLIC ${catkin_LIBRARIES} Eigen3::Eigen fmt::fmt)
target_compile_features(autoware_trajectory PUBLIC cxx_std_17)

install(TARGETS autoware_trajectory
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/autoware/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/autoware
)
