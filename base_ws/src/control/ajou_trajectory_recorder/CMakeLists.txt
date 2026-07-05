cmake_minimum_required(VERSION 3.0.2)
project(ajou_trajectory_recorder)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  morai_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    morai_msgs
    tf
)

catkin_install_python(PROGRAMS
  scripts/check_morai_connection.py
  scripts/record_ego_trajectory.py
  scripts/publish_csv_line_marker.py
  scripts/publish_recorded_trajectory.py
  scripts/trajectory_visualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
