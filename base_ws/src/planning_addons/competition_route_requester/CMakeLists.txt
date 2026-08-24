cmake_minimum_required(VERSION 3.0.2)
project(competition_route_requester)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY config docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
