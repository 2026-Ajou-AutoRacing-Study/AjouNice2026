#include "autoware/path_optimizer/node.hpp"

#include <ros/ros.h>

#include <exception>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "path_optimizer");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node("~");
  try {
    autoware::path_optimizer::PathOptimizer node(node_handle, private_node);
    ros::spin();
  } catch (const std::exception & error) {
    ROS_FATAL_STREAM("path_optimizer initialization failed: " << error.what());
    return 1;
  }
  return 0;
}
