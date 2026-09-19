#include "autoware/planning_validator_intersection_collision_checker/traffic_regulatory_priority_core_ros1.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_perception_msgs/TrafficLightElement.h>
#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>

TEST(TrafficRegulatoryPriorityCore, UsesActualRegulatoryElementIdAndFreshGroup)
{
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(
    AW149_INTERSECTION_FIXTURE_PATH);
  ASSERT_TRUE(map);
  lanelet::ConstLanelet lane;
  lanelet::Id regulatory_id = lanelet::InvalId;
  for (const auto & candidate : map->laneletLayer) {
    const auto elements = candidate.regulatoryElementsAs<lanelet::TrafficLight>();
    if (!elements.empty()) {
      lane = candidate;
      regulatory_id = elements.front()->id();
      break;
    }
  }
  ASSERT_NE(regulatory_id, lanelet::InvalId);
  autoware_perception_msgs::TrafficLightGroupArray signals;
  signals.stamp.sec = 10;
  autoware_perception_msgs::TrafficLightGroup group;
  group.traffic_light_group_id = regulatory_id;
  autoware_perception_msgs::TrafficLightElement element;
  element.shape = autoware_perception_msgs::TrafficLightElement::RIGHT_ARROW;
  element.color = autoware_perception_msgs::TrafficLightElement::GREEN;
  group.elements.push_back(element);
  signals.traffic_light_groups.push_back(group);

  EXPECT_TRUE(
    autoware::planning_validator::intersection_collision_checker::ego_turn_has_fresh_signal_priority(
      lane, &signals, ros::Time(10.1), 1.0, true));
  EXPECT_FALSE(
    autoware::planning_validator::intersection_collision_checker::ego_turn_has_fresh_signal_priority(
      lane, &signals, ros::Time(11.1), 1.0, true));
}
