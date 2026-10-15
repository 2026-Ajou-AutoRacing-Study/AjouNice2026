cmake_minimum_required(VERSION 3.0.2)
project(bevfusion_ros)

find_package(catkin REQUIRED COMPONENTS
  autoware_perception_msgs
  morai_msgs
  nav_msgs
  perception_object_pipeline
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS autoware_perception_msgs morai_msgs nav_msgs
                 perception_object_pipeline rospy sensor_msgs std_msgs
                 visualization_msgs
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_score_thresholds.py)
  catkin_add_nosetests(test/test_rotated_bev_nms.py)
endif()

catkin_install_python(
  PROGRAMS scripts/bevfusion_ros_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
