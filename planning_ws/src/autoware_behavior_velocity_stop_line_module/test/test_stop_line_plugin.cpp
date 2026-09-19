#include "manager.hpp"

#include <autoware/behavior_velocity_planner_common/experimental/plugin_interface.hpp>

#include <pluginlib/class_loader.hpp>

#include <gtest/gtest.h>

#include <algorithm>

namespace autoware::behavior_velocity_planner
{
namespace
{
TEST(StopLinePlugin, LoadsAndInitializesRos1PluginBoundary)
{
  if (!ros::isInitialized()) {
    int argc = 0;
    ros::init(argc, nullptr, "stop_line_plugin_test", ros::init_options::AnonymousName);
  }

  pluginlib::ClassLoader<PluginInterface> loader(
    "autoware_behavior_velocity_planner",
    "autoware::behavior_velocity_planner::PluginInterface");
  constexpr auto kPluginClass = "autoware::behavior_velocity_planner::StopLineModulePlugin";

  const auto declared_classes = loader.getDeclaredClasses();
  ASSERT_NE(
    std::find(declared_classes.begin(), declared_classes.end(), kPluginClass), declared_classes.end());
  ASSERT_TRUE(loader.isClassAvailable(kPluginClass));
  auto plugin = loader.createUniqueInstance(kPluginClass);
  ros::NodeHandle node;
  plugin->init(node);

  EXPECT_STREQ(plugin->getModuleName(), "stop_line");
  const auto subscriptions = plugin->getRequiredSubscriptions();
  EXPECT_FALSE(subscriptions.predicted_objects);
  EXPECT_FALSE(subscriptions.traffic_signals);
  EXPECT_FALSE(subscriptions.no_ground_pointcloud);
}

TEST(StopLinePlugin, LoadsAndInitializesExperimentalRos1PluginBoundary)
{
  if (!ros::isInitialized()) {
    int argc = 0;
    ros::init(argc, nullptr, "experimental_stop_line_plugin_test", ros::init_options::AnonymousName);
  }

  pluginlib::ClassLoader<experimental::PluginInterface> loader(
    "autoware_behavior_velocity_planner",
    "autoware::behavior_velocity_planner::experimental::PluginInterface");
  constexpr auto kPluginClass =
    "autoware::behavior_velocity_planner::experimental::StopLineModulePlugin";

  const auto declared_classes = loader.getDeclaredClasses();
  ASSERT_NE(
    std::find(declared_classes.begin(), declared_classes.end(), kPluginClass), declared_classes.end());
  ASSERT_TRUE(loader.isClassAvailable(kPluginClass));
  auto plugin = loader.createUniqueInstance(kPluginClass);
  ros::NodeHandle node;
  plugin->init(node);

  EXPECT_STREQ(plugin->getModuleName(), "stop_line");
  const auto subscriptions = plugin->getRequiredSubscriptions();
  EXPECT_FALSE(subscriptions.predicted_objects);
  EXPECT_FALSE(subscriptions.traffic_signals);
  EXPECT_FALSE(subscriptions.no_ground_pointcloud);
}
}  // namespace
}  // namespace autoware::behavior_velocity_planner

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "stop_line_plugin_test", ros::init_options::AnonymousName);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
