#include <autoware/behavior_path_bidirectional_traffic_module/manager.hpp>
#include <autoware/behavior_path_planner_common/interface/scene_module_manager_interface.hpp>

#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <ros/ros.h>

namespace
{
using autoware::behavior_path_planner::SceneModuleManagerInterface;

TEST(BidirectionalTrafficPluginRuntime, DiscoversInitializesAndCreatesIdleScene)
{
  if (!ros::isInitialized()) {
    int argc = 0;
    char ** argv = nullptr;
    ros::init(argc, argv, "aw149_bidirectional_plugin_runtime");
  }
  ros::NodeHandle node("aw149_bidirectional_plugin_runtime");
  node.setParam("bidirectional_traffic/time_to_prepare_pull_over", 1.0);
  node.setParam("bidirectional_traffic/min_distance_from_roadside", 0.2);
  node.setParam("bidirectional_traffic/keep_left_distance_from_center_line", 0.5);
  node.setParam("bidirectional_traffic/shift_distance_to_pull_over_from_center_line", 1.6);
  node.setParam("bidirectional_traffic/wait_time_for_oncoming_car", 15.0);
  node.setParam("bidirectional_traffic/max_lateral_jerk", 3.0);
  node.setParam("bidirectional_traffic/min_lateral_jerk", 2.0);

  pluginlib::ClassLoader<SceneModuleManagerInterface> loader(
    "autoware_behavior_path_planner",
    "autoware::behavior_path_planner::SceneModuleManagerInterface");
  const auto manager = loader.createInstance(
    "autoware::behavior_path_planner::BidirectionalTrafficModuleManager");
  const auto bidirectional_manager = boost::dynamic_pointer_cast<
    autoware::behavior_path_planner::BidirectionalTrafficModuleManager>(manager);

  ASSERT_NE(bidirectional_manager, nullptr);
  EXPECT_EQ(manager->name(), "bidirectional_traffic");
  ASSERT_TRUE(bidirectional_manager->init(&node));
  EXPECT_TRUE(manager->hasPlanningFactorInterface());
  EXPECT_EQ(manager->rtcInterfaceCount(), 0U);
  EXPECT_EQ(manager->markerInterfaceCount(), 0U);
  manager->updateIdleModuleInstance();
  EXPECT_NE(manager->takeIdleModule(), nullptr);
}
}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
