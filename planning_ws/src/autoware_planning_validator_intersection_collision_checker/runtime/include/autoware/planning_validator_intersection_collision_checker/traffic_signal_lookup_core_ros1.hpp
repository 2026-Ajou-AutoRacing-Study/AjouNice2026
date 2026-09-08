#ifndef AUTOWARE_PLANNING_VALIDATOR_INTERSECTION_COLLISION_CHECKER_TRAFFIC_SIGNAL_LOOKUP_CORE_ROS1_HPP_
#define AUTOWARE_PLANNING_VALIDATOR_INTERSECTION_COLLISION_CHECKER_TRAFFIC_SIGNAL_LOOKUP_CORE_ROS1_HPP_

#include <autoware_perception_msgs/TrafficLightGroupArray.h>

#include <ros/time.h>

namespace autoware::planning_validator::intersection_collision_checker_ros1
{
// Mirrors PlanningValidatorContext::get_traffic_signal: an absent, stale, or
// unmatched group is unavailable, not an invented permissive signal state.
const autoware_perception_msgs::TrafficLightGroup * find_fresh_traffic_signal_group(
  const autoware_perception_msgs::TrafficLightGroupArray * signals, int64_t group_id,
  const ros::Time & now, double timeout_seconds);
}  // namespace autoware::planning_validator::intersection_collision_checker_ros1

#endif
