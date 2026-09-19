// Copyright 2020 Tier IV, Inc. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#include "goal_pose_visualizer.hpp"

namespace autoware::mission_planner_universe
{
GoalPoseVisualizer::GoalPoseVisualizer(ros::NodeHandle & node, ros::NodeHandle & private_node)
: node_(node), private_node_(private_node)
{
  sub_route_ = node_.subscribe("input/route", 1, &GoalPoseVisualizer::echo_back_route_callback, this);
  pub_goal_pose_ = node_.advertise<geometry_msgs::PoseStamped>("output/goal_pose", 1, true);
}

void GoalPoseVisualizer::echo_back_route_callback(
  const autoware_planning_msgs::LaneletRouteConstPtr & msg)
{
  geometry_msgs::PoseStamped goal_pose;
  goal_pose.header = msg->header;
  goal_pose.pose = msg->goal_pose;
  pub_goal_pose_.publish(goal_pose);
}
}  // namespace autoware::mission_planner_universe
