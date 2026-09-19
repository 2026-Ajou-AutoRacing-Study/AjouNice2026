#include <autoware/behavior_path_planner_common/interface/scene_module_manager_interface.hpp>

#include <pluginlib/class_loader.hpp>

#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(StaticObstaclePluginDiscovery, CreatesCandidateManagerFromDeclaredDescriptor)
{
  pluginlib::ClassLoader<autoware::behavior_path_planner::SceneModuleManagerInterface> loader(
    "autoware_behavior_path_planner",
    "autoware::behavior_path_planner::SceneModuleManagerInterface");
  const auto manager = loader.createInstance(
    "autoware::behavior_path_planner::StaticObstacleAvoidanceModuleManager");
  ASSERT_TRUE(manager);
  EXPECT_EQ(manager->name(), "static_obstacle_avoidance");
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "static_obstacle_plugin_discovery");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
