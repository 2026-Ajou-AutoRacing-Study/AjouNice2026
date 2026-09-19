#include "node.hpp"
#include <ros/ros.h>
int main(int argc,char ** argv) {
  ros::init(argc,argv,"motion_velocity_planner");
  try {
    autoware::motion_velocity_planner::MotionVelocityPlannerNode node(ros::NodeHandle("~"));
    ros::spin();
  } catch(const std::exception & error) {
    ROS_FATAL("Motion Velocity Planner initialization failed: %s",error.what()); return 1;
  }
  return 0;
}
