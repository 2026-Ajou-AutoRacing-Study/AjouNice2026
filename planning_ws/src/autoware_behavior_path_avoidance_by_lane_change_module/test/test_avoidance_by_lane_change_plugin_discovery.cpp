#include <autoware/behavior_path_planner_common/interface/scene_module_manager_interface.hpp>

#include <pluginlib/class_loader.hpp>

#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(AvoidanceByLaneChangePluginDiscovery, CreatesCandidateManagerFromDeclaredDescriptor)
{
  pluginlib::ClassLoader<autoware::behavior_path_planner::SceneModuleManagerInterface> loader(
    "autoware_behavior_path_planner",
    "autoware::behavior_path_planner::SceneModuleManagerInterface");
  const auto manager = loader.createInstance(
    "autoware::behavior_path_planner::AvoidanceByLaneChangeModuleManager");
  ASSERT_TRUE(manager);
  EXPECT_EQ(manager->name(), "avoidance_by_lane_change");
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "avoidance_by_lane_change_plugin_discovery");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
