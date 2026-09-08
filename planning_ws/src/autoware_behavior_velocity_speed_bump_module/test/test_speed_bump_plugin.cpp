#include <autoware/behavior_velocity_planner_common/experimental/plugin_interface.hpp>
#include <autoware/behavior_velocity_planner_common/plugin_interface.hpp>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <ros/ros.h>

TEST(SpeedBumpPlugin, LoadsExperimentalRos1Plugin)
{
  ros::NodeHandle node;
  pluginlib::ClassLoader<autoware::behavior_velocity_planner::experimental::PluginInterface> loader(
    "autoware_behavior_velocity_planner",
    "autoware::behavior_velocity_planner::experimental::PluginInterface");
  auto plugin = loader.createUniqueInstance(
    "autoware::behavior_velocity_planner::experimental::SpeedBumpModulePlugin");
  plugin->init(node);
  EXPECT_STREQ(plugin->getModuleName(), "speed_bump");
  EXPECT_FALSE(plugin->getRequiredSubscriptions().predicted_objects);
}

TEST(SpeedBumpPlugin, LoadsNormalRos1PluginWithUpstreamParameters)
{
  ros::NodeHandle node;
  node.setParam("speed_bump/slow_start_margin", 1.0);
  node.setParam("speed_bump/slow_end_margin", 1.0);
  node.setParam("speed_bump/print_debug_info", false);
  node.setParam("speed_bump/speed_calculation/min_height", 0.05);
  node.setParam("speed_bump/speed_calculation/max_height", 0.30);
  node.setParam("speed_bump/speed_calculation/min_speed", 1.39);
  node.setParam("speed_bump/speed_calculation/max_speed", 2.78);
  pluginlib::ClassLoader<autoware::behavior_velocity_planner::PluginInterface> loader(
    "autoware_behavior_velocity_planner", "autoware::behavior_velocity_planner::PluginInterface");
  auto plugin = loader.createUniqueInstance(
    "autoware::behavior_velocity_planner::SpeedBumpModulePlugin");
  plugin->init(node);
  EXPECT_STREQ(plugin->getModuleName(), "speed_bump");
  EXPECT_FALSE(plugin->getRequiredSubscriptions().predicted_objects);
}
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test_speed_bump_plugin"); testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS();
}
