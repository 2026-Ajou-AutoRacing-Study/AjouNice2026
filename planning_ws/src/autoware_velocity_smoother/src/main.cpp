#include <autoware/velocity_smoother/node.hpp>

#include <ros/ros.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "velocity_smoother");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle("~");
  autoware::velocity_smoother::VelocitySmootherNode node(node_handle, private_node_handle);
  ros::spin();
  return 0;
}
