cmake_minimum_required(VERSION 3.0.2)
project(competition_rule_monitor)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  morai_msgs
  rospy
  std_msgs
  tf2_ros
  visualization_msgs
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
  geometry_msgs morai_msgs rospy std_msgs tf2_ros visualization_msgs
)

catkin_install_python(PROGRAMS
  scripts/competition_rule_monitor_node
  scripts/competition_rule_monitor_gui
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_rules.py)
endif()
