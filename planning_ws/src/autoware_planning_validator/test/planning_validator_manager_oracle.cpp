#include "autoware/planning_validator/planning_validator_manager.hpp"

#include <gtest/gtest.h>

#include <ros/ros.h>

namespace
{
TEST(PlanningValidatorManager, LoadsTheDeclaredTrajectoryPluginAndCallsIt)
{
  ros::NodeHandle private_nh("~");
  autoware::planning_validator::PlanningValidatorManager manager;
  ASSERT_TRUE(manager.load_plugin(private_nh, "autoware::planning_validator::TrajectoryChecker"));
  ASSERT_EQ(manager.loaded_plugin_count(), 1U);
  EXPECT_TRUE(manager.load_plugin(private_nh, "autoware::planning_validator::TrajectoryChecker"));
  EXPECT_EQ(manager.loaded_plugin_count(), 1U);
  ASSERT_TRUE(manager.load_plugin(private_nh, "autoware::planning_validator::LatencyChecker"));
  ASSERT_EQ(manager.loaded_plugin_count(), 2U);
  autoware_planning_msgs::Trajectory trajectory;
  trajectory.points.resize(2);
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    trajectory.points.at(i).pose.position.x = static_cast<double>(i) * 5.0;
    trajectory.points.at(i).pose.orientation.w = 1.0;
  }
  trajectory.header.stamp = ros::Time::now();
  const auto status = manager.validate({trajectory});
  EXPECT_TRUE(status.is_valid_size);
  EXPECT_FALSE(status.is_valid_velocity_deviation);
  EXPECT_TRUE(status.is_valid_latency);
  trajectory.header.stamp = ros::Time::now() - ros::Duration(2.0);
  const auto stale_status = manager.validate({trajectory});
  EXPECT_FALSE(stale_status.is_valid_latency);
  EXPECT_TRUE(manager.unload_plugin("autoware::planning_validator::TrajectoryChecker"));
  EXPECT_EQ(manager.loaded_plugin_count(), 1U);
  EXPECT_FALSE(manager.unload_plugin("autoware::planning_validator::TrajectoryChecker"));
  EXPECT_TRUE(manager.unload_plugin("autoware::planning_validator::LatencyChecker"));
  EXPECT_EQ(manager.loaded_plugin_count(), 0U);
}
}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "planning_validator_manager_oracle");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
