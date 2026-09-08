#include "autoware/behavior_velocity_planner/experimental/node.hpp"

#include <ros/ros.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "behavior_velocity_planner");
  ros::NodeHandle node;
  autoware::behavior_velocity_planner::experimental::BehaviorVelocityPlannerNode planner(node);
  ros::spin();
  return 0;
}
