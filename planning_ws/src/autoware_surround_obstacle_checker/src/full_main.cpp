// Copyright 2026 The Autoware Contributors
// Licensed under the Apache License, Version 2.0.
#include "node.hpp"
int main(int argc,char ** argv) {
  ros::init(argc,argv,"surround_obstacle_checker");
  try {
    autoware::surround_obstacle_checker::SurroundObstacleCheckerNode node{ros::NodeHandle("~")};
    ros::spin();
  }catch(const std::exception & error){ROS_FATAL("Surround initialization/execution failed: %s",error.what());return 1;}
  return 0;
}
