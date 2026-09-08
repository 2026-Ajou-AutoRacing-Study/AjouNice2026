#include <autoware/motion_velocity_planner/motion_velocity_planner_ros1_node.hpp>

#include <ros/ros.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "motion_velocity_planner");
  ros::NodeHandle private_node("~");
  autoware::motion_velocity_planner::MotionVelocityPlannerRos1Node node(private_node);
  ros::spin();
  return 0;
}
