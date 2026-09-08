#include <autoware/behavior_velocity_planner_common/experimental/plugin_interface.hpp>
#include <autoware/behavior_velocity_planner_common/plugin_interface.hpp>

#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <ros/master.h>
#include <ros/ros.h>
#include <tier4_v2x_msgs/VirtualTrafficLightStateArray.h>

#include <algorithm>
#include <string>

TEST(VirtualTrafficLightR028Plugin, LoadsStandardRos1Plugin)
{
  ros::NodeHandle node;
  pluginlib::ClassLoader<autoware::behavior_velocity_planner::PluginInterface> loader(
    "autoware_behavior_velocity_planner", "autoware::behavior_velocity_planner::PluginInterface");
  auto plugin = loader.createUniqueInstance(
    "autoware::behavior_velocity_planner::VirtualTrafficLightModulePlugin");
  plugin->init(node);
  EXPECT_STREQ(plugin->getModuleName(), "virtual_traffic_light");
}

TEST(VirtualTrafficLightR028Plugin, LoadsExperimentalPluginAndRegistersV2xIo)
{
  ros::NodeHandle node;
  pluginlib::ClassLoader<autoware::behavior_velocity_planner::experimental::PluginInterface> loader(
    "autoware_behavior_velocity_planner",
    "autoware::behavior_velocity_planner::experimental::PluginInterface");
  auto plugin = loader.createUniqueInstance(
    "autoware::behavior_velocity_planner::experimental::VirtualTrafficLightModulePlugin");
  plugin->init(node);
  EXPECT_STREQ(plugin->getModuleName(), "virtual_traffic_light");

  const auto input_publisher = node.advertise<tier4_v2x_msgs::VirtualTrafficLightStateArray>(
    "/input/virtual_traffic_light_states", 1);
  for (size_t i = 0; i < 20U && input_publisher.getNumSubscribers() == 0U; ++i) {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_GT(input_publisher.getNumSubscribers(), 0U);

  ros::master::V_TopicInfo topics;
  ASSERT_TRUE(ros::master::getTopics(topics));
  EXPECT_TRUE(std::any_of(topics.begin(), topics.end(), [](const auto & topic) {
    return topic.name == "/output/infrastructure_commands";
  }));
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test_vtl_r028_plugin");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
