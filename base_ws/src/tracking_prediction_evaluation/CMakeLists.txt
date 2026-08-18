cmake_minimum_required(VERSION 3.0.2)
project(tracking_prediction_evaluation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rosbag
  morai_msgs
  autoware_perception_msgs
  geometry_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/audit_bbox_reference_geometry.py
  scripts/audit_lidar_detection_timing.py
  scripts/audit_lanelet_centerline_bias.py
  scripts/audit_timestamp_alignment.py
  scripts/evaluate_oracle_prediction.py
  scripts/evaluate_pipeline_prediction.py
  scripts/retime_rolling_lidar_detections.py
  scripts/oracle_gt_tracks_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
