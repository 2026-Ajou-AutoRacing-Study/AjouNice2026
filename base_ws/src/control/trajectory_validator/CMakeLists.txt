cmake_minimum_required(VERSION 3.0.2)
project(trajectory_validator)

find_package(catkin REQUIRED COMPONENTS
  autoware_planning_msgs
  geometry_msgs
  message_generation
  morai_msgs
  nav_msgs
  rospy
  std_msgs
  visualization_msgs
)

catkin_python_setup()

add_message_files(FILES TrajectoryDebug.msg)

generate_messages(DEPENDENCIES
  autoware_planning_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    autoware_planning_msgs
    geometry_msgs
    message_runtime
    morai_msgs
    nav_msgs
    rospy
    std_msgs
    visualization_msgs
)

catkin_install_python(PROGRAMS
  scripts/trajectory_validator_node.py
  scripts/publish_history.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
