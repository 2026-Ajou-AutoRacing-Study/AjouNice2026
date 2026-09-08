#include <autoware/behavior_velocity_crosswalk_module/ros1_support.hpp>
#include <autoware/behavior_velocity_planner_common/plugin_interface.hpp>
#include <autoware/behavior_velocity_planner_common/experimental/plugin_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <gtest/gtest.h>
#include <cstdlib>

namespace bvp = autoware::behavior_velocity_planner;
template<class Interface> void checkPlugin(const std::string & base, const std::string & name)
{
  ros::NodeHandle node("/full_crosswalk_test");
  ASSERT_EQ(std::system("rosparam load \"$(rospack find autoware_behavior_velocity_crosswalk_module)/config/ros1/crosswalk.yaml\" /full_crosswalk_test"), 0);
  pluginlib::ClassLoader<Interface> loader("autoware_behavior_velocity_planner_common", base);
  auto plugin = loader.createUniqueInstance(name);
  ASSERT_TRUE(plugin);
  plugin->init(node);
  EXPECT_STREQ(plugin->getModuleName(), "crosswalk");
  const auto subscriptions = plugin->getRequiredSubscriptions();
  EXPECT_TRUE(subscriptions.predicted_objects);
  EXPECT_TRUE(subscriptions.traffic_signals);
  EXPECT_TRUE(subscriptions.occupancy_grid_map);
}
TEST(FullCrosswalk, NormalPlugin)
{
  checkPlugin<bvp::PluginInterface>("autoware::behavior_velocity_planner::PluginInterface",
    "autoware::behavior_velocity_planner::CrosswalkModulePlugin");
}
TEST(FullCrosswalk, ExperimentalPlugin)
{
  checkPlugin<bvp::experimental::PluginInterface>(
    "autoware::behavior_velocity_planner::experimental::PluginInterface",
    "autoware::behavior_velocity_planner::experimental::CrosswalkModulePlugin");
}
TEST(FullCrosswalk, FloatParameterRequiresValidValue)
{
  ros::NodeHandle node("/full_crosswalk_test/strict_float");
  node.deleteParam("value");
  EXPECT_THROW(bvp::crosswalk_ros1::require_parameter<float>(node, "value"), std::exception);
  node.setParam("value", "invalid");
  EXPECT_THROW(bvp::crosswalk_ros1::require_parameter<float>(node, "value"), std::exception);
  node.setParam("value", 1.5);
  EXPECT_FLOAT_EQ(bvp::crosswalk_ros1::require_parameter<float>(node, "value"), 1.5f);
  node.setParam("value", 1e100);
  EXPECT_THROW(bvp::crosswalk_ros1::require_parameter<float>(node, "value"), std::exception);
}
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "full_crosswalk_plugins");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
