#include <autoware/behavior_path_planner_common/interface/scene_module_manager_interface.hpp>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>

namespace
{
using autoware::behavior_path_planner::SceneModuleManagerInterface;

TEST(ExternalRequestLaneChangePlugins, LoadsBothDirectionalManagers)
{
  pluginlib::ClassLoader<SceneModuleManagerInterface> loader(
    "autoware_behavior_path_planner",
    "autoware::behavior_path_planner::SceneModuleManagerInterface");

  const auto left = loader.createInstance(
    "autoware::behavior_path_planner::ExternalRequestLaneChangeLeftModuleManager");
  const auto right = loader.createInstance(
    "autoware::behavior_path_planner::ExternalRequestLaneChangeRightModuleManager");

  EXPECT_EQ(left->name(), "external_request_lane_change_left");
  EXPECT_EQ(right->name(), "external_request_lane_change_right");
}
}  // namespace
