#include "manager.hpp"

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <cstdlib>

TEST(AvoidanceByLaneChangeManagerInit, RenderedRos1ParametersCreateIdleScene)
{
  ros::NodeHandle node;
  const std::string load_command = std::string("python3 ") + LANE_CHANGE_PARAMETER_LOADER + " /";
  ASSERT_EQ(std::system(load_command.c_str()), 0) << "lane-change ROS1 parameter adapter failed";
  autoware::behavior_path_planner::AvoidanceByLaneChangeModuleManager manager;
  ASSERT_NO_THROW(manager.init(&node));
  ASSERT_NO_THROW(manager.updateIdleModuleInstance());
  EXPECT_TRUE(manager.takeIdleModule());
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "avoidance_by_lane_change_manager_init");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
