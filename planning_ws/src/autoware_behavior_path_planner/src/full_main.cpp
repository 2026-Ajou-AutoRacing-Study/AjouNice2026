// Copyright 2026 The Autoware Contributors
// Licensed under the Apache License, Version 2.0.
#include <autoware/behavior_path_planner/behavior_path_planner_node.hpp>
int main(int argc,char ** argv) {
  ros::init(argc,argv,"behavior_path_planner");
  try {
    autoware::behavior_path_planner::BehaviorPathPlannerNode node{ros::NodeHandle("~")};
    ros::spin();
  }catch(const std::exception & error){ROS_FATAL("Full BPP failed: %s",error.what());return 1;}
  return 0;
}
