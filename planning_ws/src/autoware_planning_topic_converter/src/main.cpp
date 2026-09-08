#include <autoware/planning_topic_converter/path_to_trajectory.hpp>

#include <exception>
#include <ros/ros.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "path_to_trajectory_converter");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");
  try {
    autoware::planning_topic_converter::PathToTrajectory converter(node, private_node);
    ros::spin();
  } catch (const std::exception & exception) {
    ROS_FATAL_STREAM(exception.what());
    return 1;
  }
  return 0;
}
