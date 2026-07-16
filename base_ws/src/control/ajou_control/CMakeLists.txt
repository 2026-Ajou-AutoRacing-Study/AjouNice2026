cmake_minimum_required(VERSION 3.0.2)
project(ajou_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf
  morai_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    visualization_msgs
    tf
    morai_msgs
)

catkin_install_python(PROGRAMS
  scripts/virtual_trajectory_generator.py
  scripts/simple_morai_cmd.py
  scripts/mgeo_trajectory_generator.py
  scripts/mgeo_alignment_checker.py
  scripts/morai_map_alignment_checker.py
  scripts/ego_trajectory_comparison_visualizer.py
  scripts/validation_run_viewer.py
  scripts/pure_pursuit_pi_controller.py
  scripts/tracking_validator.py
  scripts/trajectory_follower_skeleton.py
  scripts/tracking_validator_skeleton.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
