#include "autoware/planning_validator_intersection_collision_checker/traffic_priority_core_ros1.hpp"

#include <autoware_perception_msgs/TrafficLightElement.h>
#include <gtest/gtest.h>

namespace
{
autoware_perception_msgs::TrafficLightGroup group_with(const uint8_t shape, const uint8_t color)
{
  autoware_perception_msgs::TrafficLightElement element;
  element.shape = shape;
  element.color = color;
  autoware_perception_msgs::TrafficLightGroup group;
  group.elements.push_back(element);
  return group;
}
}  // namespace

namespace icc = autoware::planning_validator::intersection_collision_checker_ros1;

TEST(TrafficPriorityCore, MissingOrWrongSignalDoesNotPrioritizeRightTurn)
{
  EXPECT_FALSE(icc::has_right_turn_priority(nullptr));
  const auto circle_green = group_with(autoware_perception_msgs::TrafficLightElement::CIRCLE,
    autoware_perception_msgs::TrafficLightElement::GREEN);
  EXPECT_FALSE(icc::has_right_turn_priority(&circle_green));
  const auto arrow_green = group_with(autoware_perception_msgs::TrafficLightElement::RIGHT_ARROW,
    autoware_perception_msgs::TrafficLightElement::GREEN);
  EXPECT_TRUE(icc::has_right_turn_priority(&arrow_green));
}

TEST(TrafficPriorityCore, LeftTurnAcceptsOnlyGreenOrAmberCircle)
{
  const auto green = group_with(autoware_perception_msgs::TrafficLightElement::CIRCLE,
    autoware_perception_msgs::TrafficLightElement::GREEN);
  const auto amber = group_with(autoware_perception_msgs::TrafficLightElement::CIRCLE,
    autoware_perception_msgs::TrafficLightElement::AMBER);
  const auto red = group_with(autoware_perception_msgs::TrafficLightElement::CIRCLE,
    autoware_perception_msgs::TrafficLightElement::RED);
  EXPECT_TRUE(icc::has_left_turn_priority(&green));
  EXPECT_TRUE(icc::has_left_turn_priority(&amber));
  EXPECT_FALSE(icc::has_left_turn_priority(&red));
}
