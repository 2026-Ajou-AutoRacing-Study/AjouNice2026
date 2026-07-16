cmake_minimum_required(VERSION 3.0.2)
project(ioniq5)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

install(
  DIRECTORY config launch meshes rviz source_assets urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/morai_sensor_static_tf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
