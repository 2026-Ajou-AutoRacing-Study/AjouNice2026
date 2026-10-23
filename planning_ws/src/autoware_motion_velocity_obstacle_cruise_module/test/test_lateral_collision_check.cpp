// Copyright 2026 AjouNice contributors
// Licensed under the Apache License, Version 2.0.
#include "lateral_collision_check.hpp"

#include <gtest/gtest.h>
#include <cmath>

namespace
{
namespace check = autoware::motion_velocity_planner::lateral_collision;
using Object = autoware_perception_msgs::PredictedObject;
using Point = autoware_planning_msgs::TrajectoryPoint;
using Vehicle = autoware::vehicle_info_utils::VehicleInfo;

geometry_msgs::Pose pose(double x, double y = 0.0, double yaw = 0.0)
{
  geometry_msgs::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.orientation.z = std::sin(yaw / 2.0);
  p.orientation.w = std::cos(yaw / 2.0);
  return p;
}

Vehicle vehicle()
{
  Vehicle v;
  v.vehicle_width_m = 2.0;
  v.max_longitudinal_offset_m = 2.0;
  v.min_longitudinal_offset_m = -1.0;
  return v;
}

nav_msgs::Odometry ego(double vx = 10.0, double vy = 0.0)
{
  nav_msgs::Odometry e;
  e.pose.pose = pose(0.0);
  e.twist.twist.linear.x = vx;
  e.twist.twist.linear.y = vy;
  return e;
}

Object object(double x, double y, double vx, double vy = 0.0, double dt = 0.1, int n = 101)
{
  Object o;
  o.shape.type = autoware_perception_msgs::Shape::BOUNDING_BOX;
  o.shape.dimensions.x = 4.0;
  o.shape.dimensions.y = 2.0;
  o.kinematics.initial_pose_with_covariance.pose = pose(x, y);
  o.kinematics.initial_twist_with_covariance.twist.linear.x = vx;
  o.kinematics.initial_twist_with_covariance.twist.linear.y = vy;
  autoware_perception_msgs::PredictedPath p;
  p.confidence = 1.0;
  p.time_step.sec = static_cast<int>(dt);
  p.time_step.nanosec = static_cast<int>(std::round((dt - p.time_step.sec) * 1e9));
  for (int i = 0; i < n; ++i) p.path.push_back(pose(x + vx * i * dt, y + vy * i * dt));
  o.kinematics.predicted_paths.push_back(p);
  return o;
}

std::vector<Point> trajectory(bool merge = false)
{
  std::vector<Point> result;
  for (int x = 0; x <= 100; ++x) {
    const double y = merge ? std::min(4.0, std::max(0.0, (x - 10) * 0.2)) : 0.0;
    Point p;
    p.pose = pose(x, y, merge && x >= 10 && x < 30 ? std::atan(0.2) : 0.0);
    p.longitudinal_velocity_mps = 100.0;  // Deliberately unlike measured ego speed.
    result.push_back(p);
  }
  return result;
}

check::PredictionOverlap overlap(
  const Object & o, const nav_msgs::Odometry & e = ego(),
  const std::vector<Point> & path = trajectory(), double age = 0.0, double horizon = 5.0,
  const Vehicle & v = vehicle(), std::size_t count = 1)
{
  return check::predicted_overlap(
    e, path, o, ros::Time(10.0 - age), ros::Time(10.0), v, horizon, count, 0, 0.5);
}

TEST(LateralRelativeMotion, EqualLateralVelocitiesDoNotClose)
{
  const auto o = object(20, 5, 5, 1);
  const auto m = check::relative_motion(ego(10, 1), o, pose(20, 5), vehicle(), 0.0);
  EXPECT_DOUBLE_EQ(m.relative_lateral_velocity_mps, 0.0);
  EXPECT_DOUBLE_EQ(m.closing_velocity_mps, 0.0);
  EXPECT_TRUE(std::isinf(m.time_to_contact_sec));
  EXPECT_FALSE(check::passes_time_gate(m, 1.0, 5.0, {}));
}

TEST(LateralRelativeMotion, EgoMotionAloneClosesOnEitherSide)
{
  for (const double side : {-1.0, 1.0}) {
    const auto o = object(20, side * 5, 5);
    const auto m = check::relative_motion(ego(10, side), o, pose(20, side * 5), vehicle(), 0.0);
    EXPECT_DOUBLE_EQ(m.relative_lateral_velocity_mps, -side);
    EXPECT_DOUBLE_EQ(m.closing_velocity_mps, 1.0);
    EXPECT_DOUBLE_EQ(m.clearance_m, 3.0);
    EXPECT_DOUBLE_EQ(m.time_to_contact_sec, 3.0);
    EXPECT_TRUE(check::passes_time_gate(m, 1.0, 5.0, {}));
  }
}

TEST(LateralRelativeMotion, MovingAwayIsNotConvertedToApproaching)
{
  const auto o = object(20, 5, 5);
  const auto m = check::relative_motion(ego(10, -1), o, pose(20, 5), vehicle(), 0.0);
  EXPECT_DOUBLE_EQ(m.closing_velocity_mps, -1.0);
  EXPECT_TRUE(std::isinf(m.time_to_contact_sec));
  EXPECT_FALSE(check::passes_time_gate(m, 1.0, 5.0, {}));
}

TEST(LateralRelativeMotion, UsesOneNormalAfterRotatingBothBodyTwists)
{
  auto e = ego(10, 1);
  e.pose.pose = pose(0, 0, M_PI_2);
  auto o = object(-5, 20, 5, -1);
  o.kinematics.initial_pose_with_covariance.pose = pose(-5, 20, M_PI_2);
  const auto m = check::relative_motion(e, o, pose(-5, 20, M_PI_2), vehicle(), M_PI_2);
  EXPECT_NEAR(m.signed_distance_m, 5.0, 1e-10);
  EXPECT_NEAR(m.relative_lateral_velocity_mps, -2.0, 1e-10);
  EXPECT_NEAR(m.time_to_contact_sec, 1.5, 1e-10);
}

TEST(LateralRelativeMotion, ProjectsAsymmetricFootprintsInDirectionOfContact)
{
  auto e = ego(0);
  e.pose.pose = pose(0, 0, M_PI_2);
  const auto left = object(0, 6, 0);
  const auto right = object(0, -6, 0);
  EXPECT_NEAR(check::relative_motion(e, left, pose(0, 6), vehicle(), 0).clearance_m, 3, 1e-10);
  EXPECT_NEAR(check::relative_motion(e, right, pose(0, -6), vehicle(), 0).clearance_m, 4, 1e-10);
}

TEST(LateralRelativeMotion, DifferentBodyFramesAndFutureYawDoNotChangeTwistFrame)
{
  auto e = ego(1);
  e.pose.pose = pose(0, 0, M_PI_2);
  auto o = object(20, 5, 0, 1);
  auto m = check::relative_motion(e, o, pose(20, 5), vehicle(), 0);
  EXPECT_NEAR(m.relative_lateral_velocity_mps, 0, 1e-10);
  o.kinematics.initial_pose_with_covariance.pose = pose(20, 5, M_PI_2);
  o.kinematics.initial_twist_with_covariance.twist.linear.x = 1;
  o.kinematics.initial_twist_with_covariance.twist.linear.y = 0;
  // The interpolated prediction may have rotated; the measured twist hasn't.
  m = check::relative_motion(e, o, pose(20, 5, 0), vehicle(), 0);
  EXPECT_NEAR(m.relative_lateral_velocity_mps, 0, 1e-10);
}

TEST(LateralPrediction, ParallelLanesRemainExcluded)
{
  const auto o = object(20, 4, 5);
  const auto future = overlap(o);
  EXPECT_FALSE(future.collision);
  const auto m = check::relative_motion(ego(), o, pose(20, 4), vehicle(), 0);
  EXPECT_FALSE(check::passes_time_gate(m, 1, 5, future));
}

TEST(LateralPrediction, PlannedMergeBypassesGateWithZeroInstantaneousRelativeSpeed)
{
  const auto o = object(20, 4, 5);
  const auto m = check::relative_motion(ego(), o, pose(20, 4), vehicle(), 0);
  EXPECT_FALSE(check::passes_time_gate(m, 1, 5, {}));
  const auto future = overlap(o, ego(), trajectory(true));
  ASSERT_TRUE(future.collision);
  EXPECT_GT(future.first_time_sec, 2.0);
  EXPECT_LT(future.first_time_sec, 4.5);
  EXPECT_TRUE(check::passes_time_gate(m, 1, 5, future));
}

TEST(LateralPrediction, SpatialCrossingAtDifferentTimesDoesNotOverlap)
{
  EXPECT_FALSE(overlap(object(20, -30, 0, 5)).collision);
  EXPECT_TRUE(overlap(object(20, -10, 0, 5)).collision);
}

TEST(LateralPrediction, HeaderAgeAlignsObjectAndEgoTime)
{
  const auto o = object(20, -20, 0, 5);
  EXPECT_FALSE(overlap(o).collision);
  EXPECT_TRUE(overlap(o, ego(), trajectory(), 2.0).collision);
}

TEST(LateralPrediction, DoesNotExtrapolateExpiredOrFuturePredictions)
{
  const auto o = object(50, 0, 0, 0, 0.1, 2);
  EXPECT_FALSE(overlap(o).collision);
  EXPECT_FALSE(overlap(o, ego(), trajectory(), 1.0).collision);
  EXPECT_FALSE(overlap(object(20, 0, 0), ego(), trajectory(), -10.0).collision);
}

TEST(LateralPrediction, UsesMeasuredSpeedInsteadOfUnconstrainedTarget)
{
  EXPECT_FALSE(overlap(object(30, 0, 0), ego(1)).collision);
  EXPECT_TRUE(overlap(object(30, 0, 0), ego(10)).collision);
}

TEST(LateralPrediction, IncludesFinalPartialIntervalAndSweptFootprints)
{
  auto v = vehicle();
  v.vehicle_width_m = 0.02;
  v.max_longitudinal_offset_m = 0.01;
  v.min_longitudinal_offset_m = -0.01;
  auto o = object(1.5, 0, 0, 0, 0.15, 2);
  o.shape.dimensions.x = 0.02;
  o.shape.dimensions.y = 0.02;
  EXPECT_TRUE(overlap(o, ego(), trajectory(), 0, 0.15, v).collision);
}

TEST(LateralPrediction, RearFootprintUsesPositiveRearOverhangArgument)
{
  EXPECT_TRUE(overlap(object(-2.5, 0, 0), ego(0)).collision);
}

TEST(LateralPrediction, RespectsPredictionConfidenceCount)
{
  auto o = object(20, 4, 5);
  auto dangerous = object(20, 0, 5).kinematics.predicted_paths.front();
  dangerous.confidence = 0.2;
  o.kinematics.predicted_paths.insert(o.kinematics.predicted_paths.begin(), dangerous);
  EXPECT_FALSE(overlap(o).collision);
  EXPECT_TRUE(overlap(o, ego(), trajectory(), 0, 5, vehicle(), 2).collision);
}

TEST(LateralPrediction, InvalidAndShortInputsDoNotProduceCollision)
{
  const auto o = object(0, 0, 0);
  EXPECT_FALSE(overlap(o, ego(), {}).collision);
  EXPECT_FALSE(overlap(o, ego(), {trajectory().front()}).collision);
  EXPECT_FALSE(overlap(o, ego(), trajectory(), 0, 0).collision);
  EXPECT_FALSE(overlap(o, ego(), trajectory(), 0, 5, vehicle(), 0).collision);
}

TEST(LateralPrediction, ReportsEarliestContactAcrossSelectedPaths)
{
  auto o = object(60, 0, -5);
  auto faster = object(60, 0, -20).kinematics.predicted_paths.front();
  faster.confidence = 0.2;
  o.kinematics.predicted_paths.push_back(faster);
  const auto primary = overlap(o);
  const auto both = overlap(o, ego(), trajectory(), 0, 5, vehicle(), 2);
  ASSERT_TRUE(primary.collision);
  ASSERT_TRUE(both.collision);
  EXPECT_LT(both.first_time_sec, primary.first_time_sec - 1.0);
}

TEST(LongitudinalPrediction, ConstantSpeedAndFinalPartialInterval)
{
  const auto samples = check::predict_longitudinal_motion(20, 0, 0, -10, 10, 1.25);
  ASSERT_FALSE(samples.empty());
  EXPECT_DOUBLE_EQ(samples.back().time_sec, 1.25);
  EXPECT_NEAR(samples.back().distance_m, 25, 1e-9);
  EXPECT_NEAR(samples.back().speed_mps, 20, 1e-9);
}

TEST(LongitudinalPrediction, RespectsJerkBeforeReachingTargetDeceleration)
{
  const auto samples = check::predict_longitudinal_motion(10, 0, -2, -1, 1, 3);
  ASSERT_FALSE(samples.empty());
  EXPECT_NEAR(samples.back().speed_mps, 6, 1e-9);
  EXPECT_NEAR(samples.back().distance_m, 25.6666666667, 1e-8);
  for (std::size_t i = 1; i < samples.size(); ++i) {
    const double dt = samples[i].time_sec - samples[i - 1].time_sec;
    EXPECT_GE(samples[i].acceleration_mps2, -2.0 - 1e-9);
    EXPECT_LE(std::abs(samples[i].acceleration_mps2 - samples[i - 1].acceleration_mps2),
      dt + 1e-9);
    EXPECT_GE(samples[i].distance_m, samples[i - 1].distance_m);
  }
}

TEST(LongitudinalPrediction, StartsFromMeasuredAccelerationWithoutInstantBraking)
{
  const auto samples = check::predict_longitudinal_motion(20, 1.5, -6, -2, 10, 0.2);
  ASSERT_FALSE(samples.empty());
  EXPECT_NEAR(samples.back().speed_mps, 20.26, 1e-9);
  EXPECT_NEAR(samples.back().acceleration_mps2, 1.1, 1e-9);
}

TEST(LongitudinalPrediction, StopsWithoutReversingOrCreeping)
{
  const auto samples = check::predict_longitudinal_motion(2, -2, -2, -10, 10, 3);
  ASSERT_FALSE(samples.empty());
  EXPECT_DOUBLE_EQ(samples.back().speed_mps, 0);
  EXPECT_NEAR(samples.back().distance_m, 1.0, 1e-9);
  for (const auto & s : samples) {
    EXPECT_GE(s.speed_mps, 0);
    EXPECT_LE(s.distance_m, 1.0 + 1e-9);
  }
}

TEST(LongitudinalPrediction, DetectsStoppingInsideJerkInterval)
{
  const auto samples = check::predict_longitudinal_motion(0.01, -1, 1, -10, 10, 1);
  ASSERT_FALSE(samples.empty());
  EXPECT_DOUBLE_EQ(samples.back().speed_mps, 0);
  EXPECT_GT(samples.back().distance_m, 0);
  EXPECT_LT(samples.back().distance_m, 0.001);
}

TEST(LongitudinalPrediction, InvalidInputsDoNotCreateMotion)
{
  EXPECT_TRUE(check::predict_longitudinal_motion(10, 0, -2, 0, 10, 1).empty());
  EXPECT_TRUE(check::predict_longitudinal_motion(10, NAN, -2, -10, 10, 1).empty());
  EXPECT_TRUE(check::predict_longitudinal_motion(-1, 0, -2, -10, 10, 1).empty());
  EXPECT_TRUE(check::predict_longitudinal_motion(10, 0, -2, -10, 10, INFINITY).empty());
}

check::PredictionOverlap with_motion(const Object & o, double speed, double min_accel, double max_accel)
{
  return check::predicted_overlap(
    ego(speed), trajectory(), o, ros::Time(10), ros::Time(10), vehicle(), 5, 1, 0, 0.5,
    check::LongitudinalMotion{0, min_accel, max_accel, -10, 10});
}

TEST(LateralPrediction, IntermediateBrakingFindsCollisionMissedByBothSpeedAndHardBraking)
{
  const auto o = object(20, -20, 0, 5);
  EXPECT_FALSE(overlap(o).collision);
  const auto hard_braking = check::predict_longitudinal_motion(10, 0, -6, -10, 10, 5);
  // The ego front plus object half-length cannot reach the crossing under hard braking.
  ASSERT_FALSE(hard_braking.empty());
  EXPECT_LT(hard_braking.back().distance_m + 2 + 2, 20);
  const auto prediction = with_motion(o, 10, -6, 0);
  EXPECT_TRUE(prediction.collision);
  EXPECT_FALSE(prediction.constant_velocity_collision);
  EXPECT_GT(prediction.motion_hypotheses, 2U);
  EXPECT_LT(prediction.collision_target_acceleration_mps2, 0);
  EXPECT_GT(prediction.collision_target_acceleration_mps2, -6);
}

TEST(LateralPrediction, AccelerationCanAlsoKeepACollisionTarget)
{
  const auto o = object(20, -10, 0, 5);
  EXPECT_FALSE(overlap(o, ego(5)).collision);
  const auto prediction = with_motion(o, 5, 0, 4);
  EXPECT_TRUE(prediction.collision);
  EXPECT_FALSE(prediction.constant_velocity_collision);
  EXPECT_GT(prediction.collision_target_acceleration_mps2, 0);
}

TEST(LateralPrediction, AdditionalMotionNeverRemovesConstantSpeedRisk)
{
  const auto prediction = with_motion(object(20, -10, 0, 5), 10, -6, 4);
  EXPECT_TRUE(prediction.collision);
  EXPECT_TRUE(prediction.constant_velocity_collision);
  EXPECT_GT(prediction.motion_hypotheses, 1U);
}

TEST(LateralPrediction, ParallelLanesDoNotOverlapUnderAccelerationOrBraking)
{
  EXPECT_FALSE(with_motion(object(20, 4, 5), 10, -6, 4).collision);
}

TEST(LateralPrediction, InvalidMotionLimitsRetainTheOriginalCheck)
{
  const auto prediction = with_motion(object(20, -10, 0, 5), 10, 1, 4);
  EXPECT_TRUE(prediction.collision);
  EXPECT_TRUE(prediction.constant_velocity_collision);
  EXPECT_EQ(prediction.motion_hypotheses, 1U);
}
}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
