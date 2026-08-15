cmake_minimum_required(VERSION 3.0.2)
project(tracking_prediction_evaluation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rosbag
  morai_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/evaluate_oracle_prediction.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
