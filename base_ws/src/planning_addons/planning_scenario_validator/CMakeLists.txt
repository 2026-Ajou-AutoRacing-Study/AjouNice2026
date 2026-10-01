cmake_minimum_required(VERSION 3.0.2)
project(planning_scenario_validator)
find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()
catkin_install_python(PROGRAMS
  scripts/planning_validator.py
  scripts/planning_validator_gui.py
  scripts/planning_validator_dashboard.py
  scripts/focus_planviz.py
  scripts/build_static_corridor_cases.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
