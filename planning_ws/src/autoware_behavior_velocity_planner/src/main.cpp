#include "autoware/behavior_velocity_planner/node.hpp"

#include <ros/ros.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "behavior_velocity_planner_node");
  ros::NodeHandle node;
  autoware::behavior_velocity_planner::BehaviorVelocityPlannerNode planner(node);
  ros::spin();
  return 0;
}
