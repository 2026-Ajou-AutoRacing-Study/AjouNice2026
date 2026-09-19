cmake_minimum_required(VERSION 3.0.2)
project(trajectory_batch_runner)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config launch manifests
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/batch_runner.py
  scripts/scenario_json_generator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
