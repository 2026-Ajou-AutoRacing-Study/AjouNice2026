#include "autoware/planning_validator/trajectory_checker_core.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace
{
autoware_planning_msgs::TrajectoryPoint point(double x, double y)
{
  autoware_planning_msgs::TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.orientation.w = 1.0;
  point.longitudinal_velocity_mps = 5.0F;
  return point;
}

TEST(TrajectoryCheckerCore, ValidatesStaticChecksAndKeepsMissingProviderChecksFailClosed)
{
  autoware::planning_validator::TrajectoryCheckerCore checker;
  autoware_planning_msgs::Trajectory trajectory;
  trajectory.points = {point(0.0, 0.0), point(5.0, 0.0), point(10.0, 0.0)};
  const auto status = checker.validate(trajectory);
  EXPECT_TRUE(status.is_valid_size);
  EXPECT_TRUE(status.is_valid_finite_value);
  EXPECT_TRUE(status.is_valid_interval);
  EXPECT_TRUE(status.is_valid_relative_angle);
  EXPECT_TRUE(status.is_valid_curvature);
  EXPECT_TRUE(status.is_valid_lateral_acc);
  EXPECT_TRUE(status.is_valid_lateral_jerk);
  EXPECT_TRUE(status.is_valid_longitudinal_max_acc);
  EXPECT_TRUE(status.is_valid_longitudinal_min_acc);
  EXPECT_TRUE(status.is_valid_steering);
  EXPECT_TRUE(status.is_valid_steering_rate);
  EXPECT_FALSE(status.is_valid_velocity_deviation);
  EXPECT_FALSE(status.is_valid_distance_deviation);
  EXPECT_FALSE(status.is_valid_intersection_collision_check);
  EXPECT_FALSE(status.is_valid_rear_collision_check);
}

TEST(TrajectoryCheckerCore, RejectsTooShortAndNonFiniteTrajectories)
{
  autoware::planning_validator::TrajectoryCheckerCore checker;
  autoware_planning_msgs::Trajectory short_trajectory;
  short_trajectory.points = {point(0.0, 0.0)};
  const auto short_status = checker.validate(short_trajectory);
  EXPECT_FALSE(short_status.is_valid_size);
  EXPECT_EQ(short_status.invalid_count, 1);
  autoware_planning_msgs::Trajectory nonfinite_trajectory;
  nonfinite_trajectory.points = {point(0.0, 0.0), point(std::numeric_limits<double>::quiet_NaN(), 0.0)};
  const auto nonfinite_status = checker.validate(nonfinite_trajectory);
  EXPECT_FALSE(nonfinite_status.is_valid_finite_value);
  EXPECT_EQ(nonfinite_status.invalid_count, 1);
}

TEST(TrajectoryCheckerCore, RejectsSourceShapedRelativeAngleAndLateralJerkBoundaries)
{
  autoware::planning_validator::TrajectoryCheckerCoreParameters parameters;
  parameters.relative_angle_threshold = 1.0;
  parameters.lateral_jerk_threshold = 0.1;
  autoware::planning_validator::TrajectoryCheckerCore checker(parameters);
  autoware_planning_msgs::Trajectory angle;
  angle.points = {point(0.0, 0.0), point(1.0, 0.0), point(1.0, 2.0)};
  EXPECT_FALSE(checker.validate(angle).is_valid_relative_angle);
  autoware_planning_msgs::Trajectory jerk;
  jerk.points = {point(0.0, 0.0), point(1.0, 0.0), point(1.0, 1.0), point(1.0, 2.0)};
  for (auto & p : jerk.points) { p.longitudinal_velocity_mps = 5.0F; p.acceleration_mps2 = 2.0F; }
  EXPECT_FALSE(checker.validate(jerk).is_valid_lateral_jerk);
}

TEST(TrajectoryCheckerCore, RejectsCurvatureBeyondConfiguredBoundary)
{
  autoware::planning_validator::TrajectoryCheckerCoreParameters parameters;
  parameters.curvature_threshold = 0.5;
  autoware::planning_validator::TrajectoryCheckerCore checker(parameters);
  autoware_planning_msgs::Trajectory trajectory;
  trajectory.points = {point(0.0, 0.0), point(1.0, 0.0), point(1.0, 1.0)};
  const auto status = checker.validate(trajectory);
  EXPECT_GT(status.max_curvature, 0.5);
  EXPECT_FALSE(status.is_valid_curvature);
}

TEST(TrajectoryCheckerCore, MatchesSourceLateralJerkMetricFixture)
{
  // Ported from test_trajectory_checker.cpp: the source fixture specifies a
  // maximum 0.9 m/s^3 lateral jerk at the final curved sample.
  autoware_planning_msgs::Trajectory trajectory;
  constexpr double spacing = 2.0;
  constexpr double radius = 10.0;
  for (size_t i = 0; i < 4; ++i) {
    auto p = point(static_cast<double>(i) * spacing, 0.0);
    p.longitudinal_velocity_mps = 1.0F;
    p.acceleration_mps2 = 1.0F;
    trajectory.points.push_back(p);
  }
  for (size_t i = 4; i <= 6; ++i) {
    const auto & last = trajectory.points.at(i - 1).pose.position;
    const double angle = static_cast<double>(i - 3) * spacing / radius;
    const double previous_angle = static_cast<double>(i - 4) * spacing / radius;
    auto p = point(
      last.x + radius * (std::sin(angle) - std::sin(previous_angle)),
      last.y + radius * ((1.0 - std::cos(angle)) - (1.0 - std::cos(previous_angle))));
    p.longitudinal_velocity_mps = static_cast<float>(i - 3);  // 1, 2, 3
    p.acceleration_mps2 = static_cast<float>(i - 3);
    trajectory.points.push_back(p);
  }
  const auto & final = trajectory.points.at(6).pose.position;
  constexpr double final_yaw = 3.0 * spacing / radius;
  for (size_t i = 7; i < 10; ++i) {
    auto p = point(
      final.x + std::cos(final_yaw) * spacing * static_cast<double>(i - 6),
      final.y + std::sin(final_yaw) * spacing * static_cast<double>(i - 6));
    p.longitudinal_velocity_mps = 3.0F;
    p.acceleration_mps2 = 3.0F;
    trajectory.points.push_back(p);
  }
  autoware::planning_validator::TrajectoryCheckerCore checker;
  const auto status = checker.validate(trajectory);
  EXPECT_NEAR(status.max_lateral_jerk, 0.9, 0.01);
}

TEST(TrajectoryCheckerCore, CalculatesEgoDependentDeviationChecksWhenKinematicsExist)
{
  autoware::planning_validator::TrajectoryCheckerCore checker;
  autoware_planning_msgs::Trajectory trajectory;
  trajectory.points = {point(0.0, 0.0), point(5.0, 0.0), point(10.0, 0.0)};
  for (auto & trajectory_point : trajectory.points) trajectory_point.longitudinal_velocity_mps = 4.0F;
  nav_msgs::Odometry kinematics;
  kinematics.pose.pose.position.x = 5.0;
  kinematics.pose.pose.position.y = 0.5;
  kinematics.twist.twist.linear.x = 4.0;
  const auto status = checker.validate({trajectory, &kinematics});
  EXPECT_TRUE(status.is_valid_velocity_deviation);
  EXPECT_TRUE(status.is_valid_distance_deviation);
  EXPECT_TRUE(status.is_valid_longitudinal_distance_deviation);
  EXPECT_DOUBLE_EQ(status.velocity_deviation, 0.0);
  EXPECT_NEAR(status.distance_deviation, -0.5, 1.0e-9);
  EXPECT_TRUE(status.is_valid_yaw_deviation);
}

TEST(TrajectoryCheckerCore, ChecksYawUnlessTheSourceHistorySaysTheTrajectoryYawDidNotJump)
{
  autoware::planning_validator::TrajectoryCheckerCore checker;
  autoware_planning_msgs::Trajectory trajectory;
  trajectory.points = {point(0.0, 0.0), point(5.0, 0.0), point(10.0, 0.0)};
  nav_msgs::Odometry kinematics;
  kinematics.pose.pose.position.x = 5.0;
  kinematics.pose.pose.orientation.z = 1.0;
  kinematics.pose.pose.orientation.w = 0.0;
  const auto without_history = checker.validate({trajectory, &kinematics});
  EXPECT_FALSE(without_history.is_valid_yaw_deviation);
  EXPECT_NEAR(without_history.yaw_deviation, 3.141592653589793, 1.0e-9);
  autoware::planning_validator::TrajectoryCheckerInput with_history{trajectory, &kinematics};
  with_history.last_valid_trajectory = &trajectory;
  const auto with_stable_history = checker.validate(with_history);
  EXPECT_TRUE(with_stable_history.is_valid_yaw_deviation);
}

TEST(TrajectoryCheckerCore, AppliesSourceForwardLengthAndShiftConditions)
{
  autoware::planning_validator::TrajectoryCheckerCore checker;
  autoware_planning_msgs::Trajectory trajectory;
  trajectory.points = {point(0.0, 1.0), point(5.0, 1.0), point(10.0, 1.0)};
  for (auto & trajectory_point : trajectory.points) trajectory_point.longitudinal_velocity_mps = 10.0F;
  nav_msgs::Odometry kinematics;
  kinematics.pose.pose.position.x = 1.0;
  kinematics.pose.pose.position.y = 0.0;
  kinematics.pose.pose.orientation.w = 1.0;
  kinematics.twist.twist.linear.x = 10.0;
  autoware_planning_msgs::Trajectory previous = trajectory;
  for (auto & trajectory_point : previous.points) trajectory_point.pose.position.y = 0.0;
  autoware::planning_validator::TrajectoryCheckerInput moving{trajectory, &kinematics};
  moving.last_valid_trajectory = &previous;
  const auto moving_status = checker.validate(moving);
  EXPECT_FALSE(moving_status.is_valid_forward_trajectory_length);
  EXPECT_FALSE(moving_status.is_valid_trajectory_shift);
  moving.vehicle_stopped = true;
  const auto stopped_status = checker.validate(moving);
  EXPECT_TRUE(stopped_status.is_valid_forward_trajectory_length);
  EXPECT_TRUE(stopped_status.is_valid_trajectory_shift);
}
}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
