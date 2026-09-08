#include <autoware/planning_validator/planning_validator_manager.hpp>

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

namespace
{
autoware_planning_msgs::Trajectory interval_trajectory()
{
  autoware_planning_msgs::Trajectory trajectory;
  trajectory.points.resize(2);
  trajectory.points.at(0).pose.orientation.w = 1.0;
  trajectory.points.at(1).pose.position.x = 200.0;
  trajectory.points.at(1).pose.orientation.w = 1.0;
  return trajectory;
}

TEST(TrajectoryCheckerConfig, AppliesRos1EnableAndThresholdParameters)
{
  ros::NodeHandle disabled_nh("~trajectory_checker_config_disabled");
  disabled_nh.setParam("trajectory_checker/interval/enable", false);
  disabled_nh.setParam("trajectory_checker/interval/threshold", 100.0);
  autoware::planning_validator::PlanningValidatorManager disabled_manager;
  ASSERT_TRUE(disabled_manager.load_plugin(
    disabled_nh, "autoware::planning_validator::TrajectoryCheckerRos1"));
  EXPECT_TRUE(disabled_manager.validate({interval_trajectory()}).is_valid_interval);

  ros::NodeHandle enabled_nh("~trajectory_checker_config_enabled");
  enabled_nh.setParam("trajectory_checker/interval/enable", true);
  enabled_nh.setParam("trajectory_checker/interval/threshold", 100.0);
  autoware::planning_validator::PlanningValidatorManager enabled_manager;
  ASSERT_TRUE(enabled_manager.load_plugin(
    enabled_nh, "autoware::planning_validator::TrajectoryCheckerRos1"));
  EXPECT_FALSE(enabled_manager.validate({interval_trajectory()}).is_valid_interval);
}

TEST(TrajectoryCheckerConfig, EscalatesSourceTrajectoryShiftHandlingAboveDefault)
{
  ros::NodeHandle nh("~trajectory_checker_handling");
  nh.setParam("default_handling_type", 0);
  nh.setParam("trajectory_checker/trajectory_shift/handling_type", 2);
  autoware::planning_validator::PlanningValidatorManager manager;
  ASSERT_TRUE(manager.load_plugin(nh, "autoware::planning_validator::TrajectoryCheckerRos1"));
  autoware_planning_msgs::Trajectory previous, shifted;
  for (int i = 0; i < 3; ++i) {
    previous.points.emplace_back(); shifted.points.emplace_back();
    previous.points.back().pose.position.x = shifted.points.back().pose.position.x = i * 5.0;
    shifted.points.back().pose.position.y = 1.0;
    previous.points.back().pose.orientation.w = shifted.points.back().pose.orientation.w = 1.0;
    previous.points.back().longitudinal_velocity_mps = shifted.points.back().longitudinal_velocity_mps = 10.0F;
  }
  nav_msgs::Odometry odom; odom.pose.pose.orientation.w = 1.0; odom.pose.pose.position.x = 1.0;
  autoware::planning_validator::TrajectoryCheckerInput input{shifted, &odom};
  input.last_valid_trajectory = &previous;
  const auto status = manager.validate(input);
  EXPECT_FALSE(status.is_valid_trajectory_shift);
  EXPECT_EQ(manager.requested_invalid_trajectory_handling(), 2);
}

TEST(TrajectoryCheckerConfig, ReadsPerCheckHandlingOverrideWithSourceMaximumRule)
{
  ros::NodeHandle nh("~trajectory_checker_per_check_handling");
  nh.setParam("default_handling_type", 0);
  nh.setParam("trajectory_checker/curvature/handling_type", 1);
  nh.setParam("trajectory_checker/curvature/threshold", 0.5);
  autoware::planning_validator::PlanningValidatorManager manager;
  ASSERT_TRUE(manager.load_plugin(nh, "autoware::planning_validator::TrajectoryCheckerRos1"));
  autoware_planning_msgs::Trajectory curved;
  curved.points.resize(3);
  curved.points.at(0).pose.orientation.w = curved.points.at(1).pose.orientation.w = curved.points.at(2).pose.orientation.w = 1.0;
  curved.points.at(1).pose.position.x = 1.0;
  curved.points.at(2).pose.position.x = 1.0; curved.points.at(2).pose.position.y = 1.0;
  const auto status = manager.validate({curved});
  EXPECT_FALSE(status.is_valid_curvature);
  EXPECT_EQ(manager.requested_invalid_trajectory_handling(), 1);
}
}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "trajectory_checker_config_oracle");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
