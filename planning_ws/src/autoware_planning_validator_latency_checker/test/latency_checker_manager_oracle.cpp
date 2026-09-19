#include <autoware/planning_validator/planning_validator_manager.hpp>

#include <gtest/gtest.h>

#include <ros/ros.h>

namespace
{
TEST(LatencyCheckerManager, LoadsExternalProviderAndAppliesLatencyResult)
{
  ros::NodeHandle private_nh("~latency_checker_manager_oracle");
  private_nh.setParam("latency_checker/enable", true);
  private_nh.setParam("latency_checker/threshold", 1.0);

  autoware::planning_validator::PlanningValidatorManager manager;
  ASSERT_TRUE(manager.load_plugin(
    private_nh, "autoware::planning_validator::LatencyCheckerRos1"));
  ASSERT_EQ(manager.loaded_plugin_count(), 1U);

  autoware_planning_msgs::Trajectory fresh;
  fresh.header.stamp = ros::Time::now();
  EXPECT_TRUE(manager.validate({fresh}).is_valid_latency);

  autoware_planning_msgs::Trajectory stale;
  stale.header.stamp = ros::Time::now() - ros::Duration(2.0);
  EXPECT_FALSE(manager.validate({stale}).is_valid_latency);
}
}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "latency_checker_manager_oracle");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
