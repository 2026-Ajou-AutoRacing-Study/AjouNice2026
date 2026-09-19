#include "manager.hpp"

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <cstdlib>

TEST(AvoidanceByLaneChangeManagerInit, RenderedRos1ParametersCreateIdleScene)
{
  ros::NodeHandle node;
  ASSERT_EQ(
    std::system(
      "rosparam load \"$(rospack find "
      "autoware_behavior_path_static_obstacle_avoidance_module)/config/"
      "static_obstacle_avoidance.ros1.yaml\" /"),
    0)
    << "static-obstacle ROS1 parameter fixture failed to load";
  ASSERT_EQ(
    std::system(
      "rosparam load \"$(rospack find "
      "autoware_launch)/config/ros1/planning/behavior_common.yaml\" /"),
    0)
    << "behavior-common ROS1 parameter fixture failed to load";
  const std::string load_command =
    std::string("python3 ") + LANE_CHANGE_PARAMETER_LOADER +
    " / --source \"$(rospack find autoware_behavior_path_lane_change_module)/config/"
    "lane_change.param.yaml\" --source \"$(rospack find "
    "autoware_behavior_path_avoidance_by_lane_change_module)/config/"
    "avoidance_by_lane_change.param.yaml\"";
  ASSERT_EQ(std::system(load_command.c_str()), 0) << "lane-change ROS1 parameter adapter failed";
  autoware::behavior_path_planner::AvoidanceByLaneChangeModuleManager manager;
  try {
    manager.init(&node);
  } catch (const std::exception & error) {
    FAIL() << "manager init rejected installed ROS1 parameters: " << error.what();
  }
  ASSERT_NO_THROW(manager.updateIdleModuleInstance());
  EXPECT_TRUE(manager.takeIdleModule());
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "avoidance_by_lane_change_manager_init");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
