#pragma once

#include <autoware_perception_msgs/TrafficLightGroupArray.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <ros/time.h>

namespace autoware::planning_validator::intersection_collision_checker
{
// Source rendering of set_right/left_turn_target_lanelets priority lambda.
bool ego_turn_has_fresh_signal_priority(
  const lanelet::ConstLanelet & ego_turn_lane,
  const autoware_perception_msgs::TrafficLightGroupArray * signals,
  const ros::Time & now, double timeout_seconds, bool right_turn);
}  // namespace autoware::planning_validator::intersection_collision_checker
