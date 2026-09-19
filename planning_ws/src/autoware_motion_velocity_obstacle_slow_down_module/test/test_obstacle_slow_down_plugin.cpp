#include <autoware/motion_velocity_planner_common/plugin_module_interface.hpp>

#include <pluginlib/class_loader.hpp>

#include <gtest/gtest.h>

TEST(ObstacleSlowDownPlugin, DiscoversCandidateRos1Plugin)
{
  pluginlib::ClassLoader<autoware::motion_velocity_planner::PluginModuleInterface> loader(
    "autoware_motion_velocity_planner",
    "autoware::motion_velocity_planner::PluginModuleInterface");
  const auto plugin = loader.createUniqueInstance(
    "autoware::motion_velocity_planner::ObstacleSlowDownModule");
  ASSERT_TRUE(plugin);
  EXPECT_EQ(plugin->getShortModuleName(), "obstacle_slow_down");
  const auto required = plugin->getRequiredSubscriptions();
  EXPECT_TRUE(required.predicted_objects);
  EXPECT_TRUE(required.no_ground_pointcloud);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "obstacle_slow_down_candidate_plugin_test", ros::init_options::AnonymousName);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
