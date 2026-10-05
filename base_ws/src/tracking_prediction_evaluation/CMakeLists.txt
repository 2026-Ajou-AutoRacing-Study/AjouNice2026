cmake_minimum_required(VERSION 3.0.2)
project(tracking_prediction_evaluation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rosbag
  morai_msgs
  autoware_perception_msgs
  builtin_interfaces
  geometry_msgs
  visualization_msgs
  std_msgs
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
  scripts/compare_vehicle_velocity_gt.py
  scripts/compare_pedestrian_velocity_gt.py
  scripts/evaluate_rosbag_detection.py
  scripts/evaluate_map_centerline_snap_ablation.py
  scripts/evaluate_velocity_oracle_ablation.py
  scripts/create_gt_velocity_oracle_detections.py
  scripts/create_prediction_gt_path_overlay.py
  scripts/filter_morai_object_region.py
  scripts/retime_rolling_lidar_detections.py
  scripts/oracle_gt_tracks_node.py
  scripts/morai_gt_bbox_visualization_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  set(NOSETESTS ${CMAKE_CURRENT_SOURCE_DIR}/test/pytest_catkin_wrapper.sh)
  catkin_add_nosetests(test)
endif()
