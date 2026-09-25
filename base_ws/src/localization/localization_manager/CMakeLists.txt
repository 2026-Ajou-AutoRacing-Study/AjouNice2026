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
  visualization_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/morai_map_camera_init_broadcaster.py
  scripts/fast_lio_kinematic_state_publisher.py
  scripts/morai_gt_kinematic_state_publisher.py
  scripts/morai_gps_to_navsatfix.py
  scripts/imu_covariance_adapter.py
  scripts/competition_yaw_rate_imu_adapter.py
  scripts/morai_gps_to_local_odometry.py
  scripts/morai_gt_motion_adapter.py
  scripts/gnss_imu_initial_pose_publisher.py
  scripts/gnss_imu_kinematic_state_publisher.py
  scripts/smooth_control_state_publisher.py
  scripts/quantized_gnss_control_state_publisher.py
  scripts/localization_relocation_guard.py
  scripts/localization_minimap_publisher.py
  scripts/localization_state_rviz_marker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_gnss_imu_contract.py)
  catkin_add_nosetests(test/test_gnss_quantization_input.py)
  catkin_add_nosetests(test/test_smooth_control_state.py)
  catkin_add_nosetests(test/test_quantized_gnss_control_state.py)
  catkin_add_nosetests(test/test_competition_yaw_rate_imu_adapter.py)
  catkin_add_nosetests(test/test_localization_relocation_guard.py)
  catkin_add_nosetests(test/test_morai_gt_contract.py)
  add_rostest(test/morai_gt_transport.test)
  add_rostest(test/localization_relocation_guard_transport.test)
endif()
