#include "autoware/planning_validator_intersection_collision_checker/traffic_priority_core_ros1.hpp"

#include <autoware/traffic_light_utils/traffic_light_utils.hpp>
#include <autoware_perception_msgs/TrafficLightElement.h>

namespace autoware::planning_validator::intersection_collision_checker_ros1
{
bool has_right_turn_priority(const autoware_perception_msgs::TrafficLightGroup * group)
{
  return group && autoware::traffic_light_utils::hasTrafficLightShapeAndColor(
    group->elements, autoware_perception_msgs::TrafficLightElement::RIGHT_ARROW,
    autoware_perception_msgs::TrafficLightElement::GREEN);
}

bool has_left_turn_priority(const autoware_perception_msgs::TrafficLightGroup * group)
{
  return group && (
    autoware::traffic_light_utils::hasTrafficLightShapeAndColor(
      group->elements, autoware_perception_msgs::TrafficLightElement::CIRCLE,
      autoware_perception_msgs::TrafficLightElement::GREEN) ||
    autoware::traffic_light_utils::hasTrafficLightShapeAndColor(
      group->elements, autoware_perception_msgs::TrafficLightElement::CIRCLE,
      autoware_perception_msgs::TrafficLightElement::AMBER));
}
}  // namespace autoware::planning_validator::intersection_collision_checker_ros1
