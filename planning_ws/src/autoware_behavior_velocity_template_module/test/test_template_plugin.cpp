#include "manager.hpp"

#include <pluginlib/class_loader.hpp>
#include <gtest/gtest.h>

#include <vector>

namespace autoware::behavior_velocity_planner
{
TEST(TemplatePlugin, LoadsExperimentalRos1PluginBoundary)
{
  pluginlib::ClassLoader<experimental::PluginInterface> loader(
    "autoware_behavior_velocity_planner",
    "autoware::behavior_velocity_planner::experimental::PluginInterface");
  constexpr auto kPlugin = "autoware::behavior_velocity_planner::TemplateModulePlugin";
  ASSERT_TRUE(loader.isClassAvailable(kPlugin));
  auto plugin = loader.createUniqueInstance(kPlugin);
  ros::NodeHandle node;
  plugin->init(node);
  EXPECT_STREQ(plugin->getModuleName(), "template");

  std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> points(2);
  points[0].point.pose.orientation.w = 1.0;
  points[1].point.pose.position.x = 1.0;
  points[1].point.pose.orientation.w = 1.0;
  auto path = experimental::Trajectory::Builder{}.build(points).value();
  PlannerData planner_data;
  plugin->updateSceneModuleInstances(path, ros::Time(42), planner_data);
  std_msgs::Header header;
  header.stamp = ros::Time(42);
  plugin->plan(path, header, {}, {}, planner_data);
  ASSERT_EQ(path.restore().size(), 2U);
  EXPECT_DOUBLE_EQ(path.restore().at(1).point.pose.position.x, 1.0);
}
}  // namespace autoware::behavior_velocity_planner

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "template_plugin_test", ros::init_options::AnonymousName);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
