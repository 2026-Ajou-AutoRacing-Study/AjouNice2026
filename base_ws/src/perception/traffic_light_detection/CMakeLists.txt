cmake_minimum_required(VERSION 3.0.2)
project(traffic_light_detection)

find_package(catkin REQUIRED COMPONENTS
  autoware_planning_msgs
  builtin_interfaces
  morai_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
  autoware_perception_msgs
)

catkin_package(CATKIN_DEPENDS
  autoware_perception_msgs
  autoware_planning_msgs
  builtin_interfaces
  morai_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
)

catkin_install_python(PROGRAMS
  scripts/lanelet_route_matcher.py
  scripts/light_color_utils.py
  scripts/signal_phase_policy.py
  scripts/signal_smoother.py
  scripts/ros_time.py
  scripts/traffic_light_node.py
  scripts/traffic_light_route_group_id_node.py
  scripts/traffic_light_map_selector.py
  scripts/traffic_light_near_rescue.py
  scripts/traffic_light_map_roi_node.py
  scripts/traffic_light_cnn_classifier.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch weights
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  README.md
  requirements-runtime.txt
  requirements-candidate.txt
  traffic_light_group_id_table.json
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
