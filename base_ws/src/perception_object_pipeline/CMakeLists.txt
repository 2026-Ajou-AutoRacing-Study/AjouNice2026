cmake_minimum_required(VERSION 3.0.2)
project(perception_object_pipeline)

find_package(catkin REQUIRED COMPONENTS
  autoware_perception_msgs
  builtin_interfaces
  geometry_msgs
  rospy
  std_msgs
  visualization_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    autoware_perception_msgs
    builtin_interfaces
    geometry_msgs
    rospy
    std_msgs
    visualization_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/object_prediction_node.py
    scripts/map_based_prediction_node.py
    scripts/object_pipeline_visualization_node.py
    scripts/tracked_objects_fixed_rate_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test)
  add_rostest(test/standard_pipeline.test)
endif()
