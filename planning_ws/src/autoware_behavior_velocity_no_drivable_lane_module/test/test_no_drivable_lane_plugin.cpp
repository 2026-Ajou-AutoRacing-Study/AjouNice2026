#include "manager.hpp"

#include <pluginlib/class_loader.hpp>

#include <gtest/gtest.h>

#include <algorithm>

namespace autoware::behavior_velocity_planner
{
namespace
{
TEST(NoDrivableLanePlugin, LoadsAndInitializesExperimentalRos1PluginBoundary)
{
  using PluginInterface = experimental::PluginInterface;
  pluginlib::ClassLoader<PluginInterface> loader(
    "autoware_behavior_velocity_planner",
    "autoware::behavior_velocity_planner::experimental::PluginInterface");
  constexpr auto kPluginClass = "autoware::behavior_velocity_planner::NoDrivableLaneModulePlugin";

  const auto declared_classes = loader.getDeclaredClasses();
  ASSERT_NE(
    std::find(declared_classes.begin(), declared_classes.end(), kPluginClass), declared_classes.end());
  ASSERT_TRUE(loader.isClassAvailable(kPluginClass));
  auto plugin = loader.createUniqueInstance(kPluginClass);
  ros::NodeHandle node;
  plugin->init(node);

  EXPECT_STREQ(plugin->getModuleName(), "no_drivable_lane");
  const auto subscriptions = plugin->getRequiredSubscriptions();
  EXPECT_FALSE(subscriptions.predicted_objects);
  EXPECT_FALSE(subscriptions.traffic_signals);
  EXPECT_FALSE(subscriptions.no_ground_pointcloud);
}
}  // namespace
}  // namespace autoware::behavior_velocity_planner

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "no_drivable_lane_plugin_test", ros::init_options::AnonymousName);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
