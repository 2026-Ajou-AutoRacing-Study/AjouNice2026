#include "autoware/behavior_velocity_planner/full_node.hpp"
int main(int argc,char ** argv) { ros::init(argc,argv,"behavior_velocity_planner_node");
try { autoware::behavior_velocity_planner::FullBehaviorVelocityPlannerNode node(ros::NodeHandle("~")); ros::spin(); }
catch(const std::exception & e) { ROS_FATAL("%s",e.what()); return 1; } return 0; }
