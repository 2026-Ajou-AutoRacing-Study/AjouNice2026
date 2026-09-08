#include "autoware/planning_validator_latency_checker/latency_checker.hpp"

#include <gtest/gtest.h>

#include <ros/ros.h>

namespace
{
autoware_planning_validator::PlanningValidatorStatus make_status()
{
  autoware_planning_validator::PlanningValidatorStatus status;
  status.latency = -123.0;
  status.is_valid_latency = false;
  return status;
}

autoware_planning_msgs::Trajectory make_trajectory(const ros::Time & stamp)
{
  autoware_planning_msgs::Trajectory trajectory;
  trajectory.header.stamp = stamp;
  return trajectory;
}

TEST(LatencyCheckerProvider, PreservesFreshStaleAndDisabledSourceBehavior)
{
  ros::NodeHandle private_nh("~latency_checker_provider_oracle");
  private_nh.setParam("latency_checker/enable", true);
  private_nh.setParam("latency_checker/threshold", 1.0);

  autoware::planning_validator::LatencyCheckerRos1 checker;
  checker.initialize(private_nh, "autoware::planning_validator::LatencyCheckerRos1");
  EXPECT_EQ(checker.module_name(), "autoware::planning_validator::LatencyCheckerRos1");

  auto fresh = make_trajectory(ros::Time::now());
  auto fresh_status = make_status();
  checker.validate({fresh}, fresh_status);
  EXPECT_TRUE(fresh_status.is_valid_latency);
  EXPECT_GE(fresh_status.latency, 0.0);
  EXPECT_LT(fresh_status.latency, 1.0);

  auto stale = make_trajectory(ros::Time::now() - ros::Duration(2.0));
  auto stale_status = make_status();
  checker.validate({stale}, stale_status);
  EXPECT_FALSE(stale_status.is_valid_latency);
  EXPECT_GT(stale_status.latency, 1.0);

  private_nh.setParam("latency_checker/enable", false);
  autoware::planning_validator::LatencyCheckerRos1 disabled_checker;
  disabled_checker.initialize(private_nh, "disabled");
  auto disabled_status = make_status();
  disabled_checker.validate({stale}, disabled_status);
  EXPECT_DOUBLE_EQ(disabled_status.latency, -123.0);
  EXPECT_FALSE(disabled_status.is_valid_latency);
}
}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "latency_checker_provider_oracle");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
