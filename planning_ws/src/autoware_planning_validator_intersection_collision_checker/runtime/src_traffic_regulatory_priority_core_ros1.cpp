#include "autoware/planning_validator_intersection_collision_checker/traffic_regulatory_priority_core_ros1.hpp"

#include "autoware/planning_validator_intersection_collision_checker/traffic_priority_core_ros1.hpp"
#include "autoware/planning_validator_intersection_collision_checker/traffic_signal_lookup_core_ros1.hpp"

#include <autoware_lanelet2_extension/regulatory_elements/Forward.hpp>

namespace autoware::planning_validator::intersection_collision_checker
{
bool ego_turn_has_fresh_signal_priority(
  const lanelet::ConstLanelet & ego_turn_lane,
  const autoware_perception_msgs::TrafficLightGroupArray * signals,
  const ros::Time & now, const double timeout_seconds, const bool right_turn)
{
  for (const auto & regulatory : ego_turn_lane.regulatoryElementsAs<lanelet::TrafficLight>()) {
    const auto * group =
      intersection_collision_checker_ros1::find_fresh_traffic_signal_group(
        signals, regulatory->id(), now, timeout_seconds);
    if (right_turn
          ? intersection_collision_checker_ros1::has_right_turn_priority(group)
          : intersection_collision_checker_ros1::has_left_turn_priority(group)) {
      return true;
    }
  }
  return false;
}
}  // namespace autoware::planning_validator::intersection_collision_checker
