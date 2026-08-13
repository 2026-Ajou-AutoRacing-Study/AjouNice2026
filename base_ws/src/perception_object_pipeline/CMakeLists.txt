cmake_minimum_required(VERSION 3.0.2)
project(perception_object_pipeline)

find_package(catkin REQUIRED COMPONENTS
  autoware_perception_msgs
  geometry_msgs
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    autoware_perception_msgs
    geometry_msgs
    rospy
    std_msgs
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
