cmake_minimum_required(VERSION 3.14)
project(autoware_global_parameter_loader)

find_package(catkin REQUIRED)

catkin_package()

# This is a launch-only package.  The dependency remains an execution
# dependency in package.xml because roslaunch resolves it when the include is
# expanded, not while this package is configured.
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
