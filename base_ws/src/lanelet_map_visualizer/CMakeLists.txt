cmake_minimum_required(VERSION 3.0.2)
project(lanelet_map_visualizer)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
  visualization_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/lanelet2_osm_marker_publisher.py
  scripts/mgeo_json_marker_publisher.py
  scripts/validate_lanelet_projector.py
  map_edit/lanelet_map_builder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
