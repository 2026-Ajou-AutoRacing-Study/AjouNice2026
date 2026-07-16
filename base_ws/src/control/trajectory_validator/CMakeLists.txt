cmake_minimum_required(VERSION 3.0.2)
project(trajectory_validator)

find_package(catkin REQUIRED COMPONENTS
  message_generation rospy std_msgs geometry_msgs nav_msgs visualization_msgs tf2_ros tf2_geometry_msgs
  autoware_planning_msgs morai_msgs rqt_gui rqt_gui_py
)

add_service_files(FILES SetHistoryVisibility.srv)
catkin_python_setup()
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime rospy std_msgs geometry_msgs nav_msgs visualization_msgs tf2_ros tf2_geometry_msgs autoware_planning_msgs morai_msgs)

catkin_install_python(PROGRAMS scripts/trajectory_validator_node.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch history DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)
install(FILES plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
