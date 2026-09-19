#include <autoware/behavior_path_planner/behavior_path_planner_ros1_node.hpp>

#include <ros/ros.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "behavior_path_planner");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");
  autoware::behavior_path_planner::BehaviorPathPlannerRos1Node planner(node, private_node);
  ros::spin();
  return 0;
}
