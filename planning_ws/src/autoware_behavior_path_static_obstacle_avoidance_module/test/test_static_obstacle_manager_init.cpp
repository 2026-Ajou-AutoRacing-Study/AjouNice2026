#include <autoware/behavior_path_static_obstacle_avoidance_module/manager.hpp>

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <cstdlib>

TEST(StaticObstacleManagerInit, CompleteRos1ParameterSurfaceCreatesIdleScene)
{
  ros::NodeHandle node;
  ASSERT_EQ(
    std::system(
      "rosparam load \"$(rospack find "
      "autoware_behavior_path_static_obstacle_avoidance_module)/config/"
      "static_obstacle_avoidance.ros1.yaml\" /"),
    0)
    << "static-obstacle ROS1 parameter fixture failed to load";
  // Preserve the explicit policy oracle while sourcing the remaining surface
  // from the same installed YAML used by roslaunch.
  node.setParam("avoidance/policy/make_approval_request", "per_shift_line");
  node.setParam("avoidance/policy/deceleration", "best_effort");
  node.setParam("avoidance/policy/lateral_margin", "best_effort");
  node.setParam("avoidance/policy/candidate_path_turn_signal", "none");
  autoware::behavior_path_planner::StaticObstacleAvoidanceModuleManager manager;
  try {
    manager.init(&node);
  } catch (const std::exception & error) {
    FAIL() << "manager init rejected rendered ROS1 parameters: " << error.what();
  }
  ASSERT_NO_THROW(manager.updateIdleModuleInstance());
  EXPECT_TRUE(manager.takeIdleModule());
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "static_obstacle_manager_init");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
