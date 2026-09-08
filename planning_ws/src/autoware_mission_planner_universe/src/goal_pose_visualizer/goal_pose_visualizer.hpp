// Copyright 2020 Tier IV, Inc. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#ifndef GOAL_POSE_VISUALIZER__GOAL_POSE_VISUALIZER_HPP_
#define GOAL_POSE_VISUALIZER__GOAL_POSE_VISUALIZER_HPP_

#include <ros/ros.h>

#include <autoware_planning_msgs/LaneletRoute.h>
#include <geometry_msgs/PoseStamped.h>

namespace autoware::mission_planner_universe
{
class GoalPoseVisualizer
{
public:
  GoalPoseVisualizer(ros::NodeHandle & node, ros::NodeHandle & private_node);

private:
  ros::NodeHandle & node_;
  ros::NodeHandle & private_node_;
  ros::Subscriber sub_route_;
  ros::Publisher pub_goal_pose_;

  void echo_back_route_callback(const autoware_planning_msgs::LaneletRouteConstPtr & msg);
};
}  // namespace autoware::mission_planner_universe

#endif  // GOAL_POSE_VISUALIZER__GOAL_POSE_VISUALIZER_HPP_
