#pragma once

#include <autoware_perception_msgs/TrafficLightGroup.h>

namespace autoware::planning_validator::intersection_collision_checker_ros1
{
// Exact source conditions in set_right/left_turn_target_lanelets.  A missing
// group is deliberately not prioritized: absence must retain collision checks.
bool has_right_turn_priority(const autoware_perception_msgs::TrafficLightGroup * group);
bool has_left_turn_priority(const autoware_perception_msgs::TrafficLightGroup * group);
}  // namespace autoware::planning_validator::intersection_collision_checker_ros1
