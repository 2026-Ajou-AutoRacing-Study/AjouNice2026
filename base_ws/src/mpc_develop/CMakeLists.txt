cmake_minimum_required(VERSION 3.0.2)
project(mpc_develop)

find_package(catkin REQUIRED COMPONENTS
  morai_msgs
  rosbag
  rosgraph
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS morai_msgs rosbag rosgraph rospy std_msgs
)

catkin_install_python(PROGRAMS
  scripts/automatic_excitation_publisher.py
  scripts/bag_to_lateral_id_csv.py
  scripts/manual_command_publisher.py
  scripts/record_identification_bag.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY matlab
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
