cmake_minimum_required(VERSION 3.0.2)
project(ajounice_planning_integration)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/fixed_autonomous_operation_mode_publisher
  scripts/map_projector_publisher
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_vehicle_info_contract.py)
  catkin_add_nosetests(test/test_morai_stack_perception_contract.py)
  catkin_add_nosetests(test/test_map_profile_contract.py)
  catkin_add_nosetests(test/test_signal_yellow_braking_launch.py)
  add_rostest(test/fixed_autonomous_operation_mode.test)
  add_rostest(test/operation_mode_relay.test)
endif()
