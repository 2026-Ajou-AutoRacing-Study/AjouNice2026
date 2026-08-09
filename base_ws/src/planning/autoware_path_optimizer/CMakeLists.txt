cmake_minimum_required(VERSION 3.0.2)
project(autoware_path_optimizer)

find_package(catkin REQUIRED COMPONENTS
  autoware_internal_debug_msgs
  autoware_perception_msgs
  autoware_planning_msgs
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  visualization_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  autoware_internal_debug_msgs
  autoware_perception_msgs
  autoware_planning_msgs
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  visualization_msgs
)

catkin_install_python(
  PROGRAMS
  scripts/path_optimizer.py
  validation/scripts/path_optimizer_reference_validator.py
  validation/scripts/path_optimizer_comparison_visualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY src/autoware_path_optimizer
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

install(
  DIRECTORY validation/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/validation
)

install(
  FILES docs/AUTOWARE_PATH_OPTIMIZER_PORT.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/docs
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_mpt_optimizer.py)
  catkin_add_nosetests(test/test_replan_checker.py)
  catkin_add_nosetests(test/test_static_collision_checker.py)
endif()
