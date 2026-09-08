#include <autoware/behavior_path_static_obstacle_avoidance_module/manager.hpp>

#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(StaticObstacleManagerInit, CompleteRos1ParameterSurfaceCreatesIdleScene)
{
  ros::NodeHandle node;
  // The manager reads the rendered ROS1 tree directly.  This fixture is not a
  // roslaunch process, so provide the required policy leaves from the staged
  // static_obstacle_avoidance.ros1.yaml explicitly.
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
