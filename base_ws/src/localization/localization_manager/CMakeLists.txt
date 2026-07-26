cmake_minimum_required(VERSION 3.0.2)
project(localization_manager)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  morai_msgs
  nav_msgs
  robot_localization
  rospy
  sensor_msgs
  tf
  tf2_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/morai_map_camera_init_broadcaster.py
  scripts/fast_lio_kinematic_state_publisher.py
  scripts/morai_gt_kinematic_state_publisher.py
  scripts/morai_gps_to_navsatfix.py
  scripts/imu_covariance_adapter.py
  scripts/morai_gps_to_local_odometry.py
  scripts/gnss_imu_kinematic_state_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)
