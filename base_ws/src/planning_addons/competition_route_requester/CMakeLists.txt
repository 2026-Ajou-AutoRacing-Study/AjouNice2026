cmake_minimum_required(VERSION 3.0.2)
project(competition_route_requester)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs geometry_msgs nav_msgs
  message_generation autoware_adapi_v1_msgs autoware_internal_planning_msgs
  autoware_map_msgs autoware_planning_msgs autoware_planning_validator tier4_planning_msgs)
catkin_python_setup()
add_service_files(FILES RequestRoute.srv)
generate_messages()
catkin_package(CATKIN_DEPENDS message_runtime rospy std_msgs nav_msgs geometry_msgs
  autoware_adapi_v1_msgs autoware_internal_planning_msgs autoware_map_msgs
  autoware_planning_msgs autoware_planning_validator tier4_planning_msgs)
catkin_install_python(PROGRAMS scripts/route_requester scripts/route_supervisor scripts/finish_guard
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(
  DIRECTORY config docs launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_policy.py)
  catkin_add_nosetests(test/test_v94_profile.py)
  catkin_add_nosetests(test/test_v98_profile.py)
endif()

install(
  FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
