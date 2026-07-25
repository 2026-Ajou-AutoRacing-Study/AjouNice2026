#include "trajectory_follower/adaptive_pure_pursuit_core.hpp"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

namespace
{
std::vector<trajectory_follower::PathPoint> straightPath(const double velocity)
{
  std::vector<trajectory_follower::PathPoint> path;
  for (int i = 0; i <= 80; ++i) {
    trajectory_follower::PathPoint point;
    point.pose.x = static_cast<double>(i);
    point.pose.y = 0.0;
    point.pose.yaw = 0.0;
    point.target_velocity_mps = velocity;
    path.push_back(point);
  }
  return path;
}

trajectory_follower::VehicleState centeredState(const double velocity)
{
  trajectory_follower::VehicleState state;
  state.pose.x = 1.0;
  state.pose.y = 0.0;
  state.pose.yaw = 0.0;
  state.velocity_mps = velocity;
  return state;
}
}  // namespace

TEST(AdaptivePurePursuitCore, StraightPathUsesAutowareMaximumLookahead)
{
  trajectory_follower::PurePursuitParams params;
  trajectory_follower::AdaptivePurePursuitCore core(params);
  const auto output = core.run(straightPath(10.0), centeredState(10.0));
  EXPECT_TRUE(output.valid);
  EXPECT_NEAR(output.velocity_ld, 24.0, 1e-9);
  EXPECT_NEAR(output.lookahead_distance, 15.0, 1e-9);
  EXPECT_NEAR(output.target_curvature, 0.0, 1e-9);
  EXPECT_NEAR(output.steering_tire_angle_rad, 0.0, 1e-9);
}

TEST(AdaptivePurePursuitCore, LateralOffsetHasExpectedSign)
{
  trajectory_follower::PurePursuitParams params;
  trajectory_follower::AdaptivePurePursuitCore core(params);
  auto left = centeredState(0.0);
  left.pose.y = 1.0;
  const auto left_output = core.run(straightPath(0.0), left);
  EXPECT_TRUE(left_output.valid);
  EXPECT_GT(left_output.lateral_error, 0.0);
  EXPECT_LT(left_output.steering_tire_angle_rad, 0.0);

  trajectory_follower::AdaptivePurePursuitCore right_core(params);
  auto right = centeredState(0.0);
  right.pose.y = -1.0;
  const auto right_output = right_core.run(straightPath(0.0), right);
  EXPECT_TRUE(right_output.valid);
  EXPECT_LT(right_output.lateral_error, 0.0);
  EXPECT_GT(right_output.steering_tire_angle_rad, 0.0);
}

TEST(AdaptivePurePursuitCore, LargeLateralOffsetKeepsControllerAvailableForRecovery)
{
  trajectory_follower::PurePursuitParams params;
  params.lookahead_mode = trajectory_follower::LookaheadMode::kFixed;
  params.fixed_lookahead_distance = 1.9;
  trajectory_follower::AdaptivePurePursuitCore core(params);
  auto state = centeredState(0.0);
  state.pose.y = 5.0;

  const auto output = core.run(straightPath(0.0), state);

  EXPECT_TRUE(output.valid);
  EXPECT_GT(std::abs(output.lateral_error), params.fixed_lookahead_distance);
  EXPECT_TRUE(output.failure_reason.empty());
  EXPECT_LT(output.steering_tire_angle_rad, 0.0);
}

TEST(AdaptivePurePursuitCore, LargeHeadingErrorKeepsControllerAvailableForRecovery)
{
  trajectory_follower::PurePursuitParams params;
  params.lookahead_mode = trajectory_follower::LookaheadMode::kFixed;
  params.fixed_lookahead_distance = 4.0;
  trajectory_follower::AdaptivePurePursuitCore core(params);
  auto state = centeredState(0.0);
  state.pose.yaw = M_PI_2;

  const auto output = core.run(straightPath(0.0), state);

  EXPECT_TRUE(output.valid);
  EXPECT_TRUE(output.failure_reason.empty());
}

TEST(AdaptivePurePursuitCore, ReverseUsesReverseMinimumLookahead)
{
  trajectory_follower::PurePursuitParams params;
  trajectory_follower::AdaptivePurePursuitCore core(params);
  auto path = straightPath(-1.0);
  for (auto & point : path) {
    point.pose.yaw = M_PI;
  }
  auto state = centeredState(-1.0);
  state.pose.yaw = M_PI;
  const auto output = core.run(path, state);
  EXPECT_TRUE(output.valid);
  EXPECT_NEAR(output.lookahead_distance, params.reverse_min_lookahead_distance, 1e-9);
}

TEST(AdaptivePurePursuitCore, ReverseTargetUsesTrajectoryPoseDirection)
{
  trajectory_follower::PurePursuitParams params;
  trajectory_follower::AdaptivePurePursuitCore core(params);
  auto path = straightPath(-1.0);
  for (auto & point : path) {
    point.pose.yaw = M_PI;
  }
  auto state = centeredState(-1.0);
  state.pose.yaw = M_PI;
  const auto output = core.run(path, state);
  ASSERT_TRUE(output.valid);
  // Autoware selects reverse points using the pose orientation, then interpolates in the
  // trajectory's point order. The target is therefore at a larger map x here.
  EXPECT_GT(output.target_point.x, state.pose.x);
}

TEST(AdaptivePurePursuitCore, SaturatesSteering)
{
  trajectory_follower::PurePursuitParams params;
  params.max_steering_angle = 0.1;
  trajectory_follower::AdaptivePurePursuitCore core(params);
  std::vector<trajectory_follower::PathPoint> path;
  for (int i = 0; i <= 20; ++i) {
    const double theta = 0.1 * static_cast<double>(i);
    trajectory_follower::PathPoint point;
    point.pose.x = 10.0 * std::sin(theta);
    point.pose.y = 10.0 * (1.0 - std::cos(theta));
    point.pose.yaw = theta;
    point.target_velocity_mps = 2.0;
    path.push_back(point);
  }
  auto state = centeredState(2.0);
  state.pose.x = 0.0;
  const auto output = core.run(path, state);
  EXPECT_TRUE(output.valid);
  EXPECT_NEAR(output.steering_tire_angle_rad, 0.1, 1e-9);
}

TEST(AdaptivePurePursuitCore, RejectsShortPathSafely)
{
  trajectory_follower::PurePursuitParams params;
  trajectory_follower::AdaptivePurePursuitCore core(params);
  std::vector<trajectory_follower::PathPoint> path(2);
  const auto output = core.run(path, centeredState(0.0));
  EXPECT_FALSE(output.valid);
  EXPECT_EQ(output.steering_tire_angle_rad, 0.0);
}

TEST(AdaptivePurePursuitCore, FixedModeUsesExactDistanceAtEverySpeed)
{
  trajectory_follower::PurePursuitParams params;
  params.lookahead_mode = trajectory_follower::LookaheadMode::kFixed;
  params.fixed_lookahead_distance = 4.05;
  trajectory_follower::AdaptivePurePursuitCore core(params);

  const auto slow = core.run(straightPath(2.0), centeredState(2.0));
  const auto fast = core.run(straightPath(25.0), centeredState(25.0));

  ASSERT_TRUE(slow.valid);
  ASSERT_TRUE(fast.valid);
  EXPECT_DOUBLE_EQ(slow.lookahead_distance, 4.05);
  EXPECT_DOUBLE_EQ(fast.lookahead_distance, 4.05);
}

TEST(AdaptivePurePursuitCore, PathCurvatureAndSteeringAreModeIndependentAtEqualLd)
{
  std::vector<trajectory_follower::PathPoint> path;
  for (int i = 0; i <= 100; ++i) {
    const double theta = 0.01 * static_cast<double>(i);
    trajectory_follower::PathPoint point;
    point.pose.x = 20.0 * std::sin(theta);
    point.pose.y = 20.0 * (1.0 - std::cos(theta));
    point.pose.yaw = theta;
    point.target_velocity_mps = 5.0;
    path.push_back(point);
  }
  auto state = centeredState(5.0);
  state.pose = path.front().pose;

  trajectory_follower::PurePursuitParams adaptive_params;
  adaptive_params.min_lookahead_distance = 4.0;
  adaptive_params.max_lookahead_distance = 4.0;
  adaptive_params.reverse_min_lookahead_distance = 4.0;
  trajectory_follower::AdaptivePurePursuitCore adaptive(adaptive_params);

  trajectory_follower::PurePursuitParams fixed_params = adaptive_params;
  fixed_params.lookahead_mode = trajectory_follower::LookaheadMode::kFixed;
  fixed_params.fixed_lookahead_distance = 4.0;
  trajectory_follower::AdaptivePurePursuitCore fixed(fixed_params);

  const auto adaptive_output = adaptive.run(path, state);
  const auto fixed_output = fixed.run(path, state);
  ASSERT_TRUE(adaptive_output.valid);
  ASSERT_TRUE(fixed_output.valid);
  EXPECT_NEAR(adaptive_output.path_curvature, 0.05, 1e-3);
  EXPECT_DOUBLE_EQ(adaptive_output.path_curvature, fixed_output.path_curvature);
  EXPECT_DOUBLE_EQ(adaptive_output.lookahead_distance, fixed_output.lookahead_distance);
  EXPECT_DOUBLE_EQ(adaptive_output.target_curvature, fixed_output.target_curvature);
  EXPECT_DOUBLE_EQ(
    adaptive_output.steering_tire_angle_rad, fixed_output.steering_tire_angle_rad);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
