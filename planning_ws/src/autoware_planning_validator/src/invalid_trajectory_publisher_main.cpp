#include <autoware/planning_validator/invalid_trajectory_publisher.hpp>
#include <ros/ros.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "invalid_trajectory_publisher");
  ros::NodeHandle private_node("~");
  autoware::planning_validator::InvalidTrajectoryPublisher node(private_node);
  ros::spin();
  return 0;
}
