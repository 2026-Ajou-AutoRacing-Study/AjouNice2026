#include <autoware/planning_validator_intersection_collision_checker/traffic_signal_lookup_core_ros1.hpp>

#include <gtest/gtest.h>

namespace icc = autoware::planning_validator::intersection_collision_checker_ros1;

TEST(TrafficSignalLookupCoreRos1, FindsOnlyFreshMatchingGroup)
{
  autoware_perception_msgs::TrafficLightGroupArray signals;
  signals.stamp.sec = 10;
  signals.stamp.nanosec = 0;
  autoware_perception_msgs::TrafficLightGroup group;
  group.traffic_light_group_id = 42;
  signals.traffic_light_groups.push_back(group);
  EXPECT_NE(icc::find_fresh_traffic_signal_group(&signals, 42, ros::Time(10.2), 0.5), nullptr);
  EXPECT_EQ(icc::find_fresh_traffic_signal_group(&signals, 99, ros::Time(10.2), 0.5), nullptr);
  EXPECT_EQ(icc::find_fresh_traffic_signal_group(&signals, 42, ros::Time(10.6), 0.5), nullptr);
}

TEST(TrafficSignalLookupCoreRos1, MissingInputIsUnavailable)
{
  EXPECT_EQ(icc::find_fresh_traffic_signal_group(nullptr, 42, ros::Time(10.0), 0.5), nullptr);
}
