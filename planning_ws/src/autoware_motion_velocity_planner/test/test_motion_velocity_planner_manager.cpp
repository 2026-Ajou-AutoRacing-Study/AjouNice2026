#include <autoware/motion_velocity_planner/motion_velocity_planner_manager.hpp>

#include <gtest/gtest.h>

#include <cstdlib>

TEST(MotionVelocityPlannerManager, UnavailablePluginIsRejectedWithoutAnEmptyRegistration)
{
  ros::NodeHandle node("/mvp_plugin_manager_oracle");
  autoware::motion_velocity_planner::MotionVelocityPlannerManager manager;
  EXPECT_FALSE(manager.launchModule(node, "autoware::motion_velocity_planner::MissingModule"));
  EXPECT_EQ(manager.size(), 0U);
  const auto & required = manager.requiredSubscriptions();
  EXPECT_FALSE(required.traffic_signals);
  EXPECT_FALSE(required.predicted_objects);
  EXPECT_FALSE(required.occupancy_grid_map);
  EXPECT_FALSE(required.no_ground_pointcloud);
}

TEST(MotionVelocityPlannerManager, LoadsObstacleStopPluginAndPropagatesObjectRequirement)
{
  ros::NodeHandle node("/mvp_obstacle_stop_plugin_oracle");
  ASSERT_EQ(
    std::system(
      "rosparam load \"$(rospack find autoware_motion_velocity_planner)/config/ros1/"
      "obstacle_stop_default.yaml\" /mvp_obstacle_stop_plugin_oracle"),
    0);
  ASSERT_EQ(
    std::system(
      "rosparam load \"$(rospack find autoware_velocity_smoother)/config/ros1/"
      "default_common.yaml\" /mvp_obstacle_stop_plugin_oracle"),
    0);
  ASSERT_EQ(
    std::system(
      "rosparam load \"$(rospack find autoware_velocity_smoother)/config/ros1/"
      "Analytical.yaml\" /mvp_obstacle_stop_plugin_oracle"),
    0);
  autoware::motion_velocity_planner::MotionVelocityPlannerManager manager;
  EXPECT_TRUE(manager.launchModule(
    node, "autoware::motion_velocity_planner::ObstacleStopModule"));
  EXPECT_EQ(manager.size(), 1U);
  EXPECT_TRUE(manager.requiredSubscriptions().predicted_objects);
  EXPECT_TRUE(manager.requiredSubscriptions().no_ground_pointcloud);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "motion_velocity_planner_manager_test");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
