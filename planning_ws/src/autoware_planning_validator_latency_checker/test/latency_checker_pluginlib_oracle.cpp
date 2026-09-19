#include <autoware/planning_validator/plugin_interface.hpp>

#include <gtest/gtest.h>

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

namespace
{
TEST(LatencyCheckerPluginlib, LoadsInstalledExternalProvider)
{
  ros::NodeHandle private_nh("~latency_checker_pluginlib_oracle");
  private_nh.setParam("latency_checker/enable", true);
  private_nh.setParam("latency_checker/threshold", 1.0);

  pluginlib::ClassLoader<autoware::planning_validator::PluginInterface> loader(
    "autoware_planning_validator", "autoware::planning_validator::PluginInterface");
  ASSERT_TRUE(loader.isClassAvailable("autoware::planning_validator::LatencyCheckerRos1"));
  const auto checker = loader.createInstance("autoware::planning_validator::LatencyCheckerRos1");
  checker->initialize(private_nh, "autoware::planning_validator::LatencyCheckerRos1");

  autoware_planning_msgs::Trajectory trajectory;
  trajectory.header.stamp = ros::Time::now();
  autoware_planning_validator::PlanningValidatorStatus status;
  checker->validate({trajectory}, status);
  EXPECT_TRUE(status.is_valid_latency);
}
}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "latency_checker_pluginlib_oracle");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
