// Copyright 2026 AjouNice2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0

#include "autoware/path_optimizer/mpt_optimizer.hpp"
#include "autoware/path_optimizer/history_compatibility.hpp"
#include "autoware/path_optimizer/utils/geometry_utils.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include <stdexcept>

namespace autoware::path_optimizer
{
TEST(MptHistoryJoin, PreservesStraightAndFeasibleCurve)
{
  std::vector<geometry_msgs::Point> points(30);
  const auto position = [](const auto & p) { return p; };
  for (size_t i = 0; i < points.size(); ++i) points[i].x = 0.5 * i;
  EXPECT_TRUE(detail::historyPrefixCompatible(points, points, 0.28, 0.5, position));
  for (size_t i = 0; i < points.size(); ++i) {
    points[i].x = 10.0 * std::cos(0.05 * i);
    points[i].y = 10.0 * std::sin(0.05 * i);
  }
  EXPECT_TRUE(detail::historyPrefixCompatible(points, points, 0.28, 0.5, position));
}

TEST(MptHistoryJoin, RejectsRecordedLateralSpliceButNotFreshInputDefect)
{
  std::vector<geometry_msgs::Point> fresh(30), candidate;
  for (size_t i = 0; i < fresh.size(); ++i) {
    fresh[i].x = 196.825; fresh[i].y = 662.5 - 0.5 * i;
  }
  candidate = fresh;
  const double xy[][2] = {{198.406778,662.429035},{198.406187,661.929035},
    {197.973022,661.783997},{197.433407,661.697467},{196.992776,661.460154},
    {196.824816,660.990116}};
  for (size_t i = 0; i < 6; ++i) { candidate[i].x = xy[i][0]; candidate[i].y = xy[i][1]; }
  const auto position = [](const auto & p) { return p; };
  EXPECT_FALSE(detail::historyPrefixCompatible(candidate, fresh, 0.28, 0.5, position));
  EXPECT_TRUE(detail::historyPrefixCompatible(candidate, candidate, 0.28, 0.5, position));
  EXPECT_FALSE(detail::historyPrefixCompatible(candidate, fresh, 0.0, 0.5, position));
}

TEST(MptMeasuredAnchor, ReplacesHistoryButKeepsCorridorAndFutureState)
{
  std::vector<ReferencePoint> points(3);
  for (auto & p : points) {
    p.pose.orientation.w = 1.0;
    p.bounds = Bounds{-1.0, 2.0};
    p.fixed_kinematic_state = KinematicState{0.3, 0.1};
  }
  geometry_msgs::Pose ego;
  ego.position.y = 0.25;
  ego.orientation.w = std::cos(0.1);
  ego.orientation.z = std::sin(0.1);
  detail::anchorMeasuredEgo(points, 1, ego);
  EXPECT_FALSE(points[0].fixed_kinematic_state);
  ASSERT_TRUE(points[1].fixed_kinematic_state);
  EXPECT_NEAR(points[1].fixed_kinematic_state->lat, 0.25, 1e-12);
  EXPECT_NEAR(points[1].fixed_kinematic_state->yaw, 0.2, 1e-12);
  EXPECT_DOUBLE_EQ(points[2].fixed_kinematic_state->lat, 0.3);
  for (const auto & p : points) {
    EXPECT_DOUBLE_EQ(p.bounds.lower_bound, -1.0);
    EXPECT_DOUBLE_EQ(p.bounds.upper_bound, 2.0);
    EXPECT_DOUBLE_EQ(p.pose.position.y, 0.0);
  }
  EXPECT_THROW(detail::anchorMeasuredEgo(points, 3, ego), std::out_of_range);
}

TEST(MptMeasuredAnchor, UsesReferenceFrameAndWrappedYaw)
{
  std::vector<ReferencePoint> points(1);
  points[0].pose.orientation.z = std::sin(M_PI/4);
  points[0].pose.orientation.w = std::cos(M_PI/4);
  geometry_msgs::Pose ego;
  ego.position.x = 2.0;
  ego.orientation = points[0].pose.orientation;
  detail::anchorMeasuredEgo(points, 0, ego);
  EXPECT_NEAR(points[0].fixed_kinematic_state->lat, -2.0, 1e-12);
  EXPECT_NEAR(points[0].fixed_kinematic_state->yaw, 0.0, 1e-12);
  points[0].pose.orientation.z = std::sin((M_PI-0.01)/2);
  points[0].pose.orientation.w = std::cos((M_PI-0.01)/2);
  ego.orientation.z = std::sin((-M_PI+0.01)/2);
  ego.orientation.w = std::cos((-M_PI+0.01)/2);
  detail::anchorMeasuredEgo(points, 0, ego);
  EXPECT_NEAR(points[0].fixed_kinematic_state->yaw, 0.02, 1e-12);
}

TEST(MptBoundsInterpolation, SaturatesBeforeUnsignedSubtraction)
{
  EXPECT_EQ(detail::boundsInterpolationPreviousIndex(0, 40), 0U);
  EXPECT_EQ(detail::boundsInterpolationPreviousIndex(1, 40), 0U);
  EXPECT_EQ(detail::boundsInterpolationPreviousIndex(39, 40), 38U);
  EXPECT_EQ(detail::boundsInterpolationPreviousIndex(40, 40), 38U);
  EXPECT_EQ(detail::boundsInterpolationPreviousIndex(0, 2), 0U);
  EXPECT_EQ(detail::boundsInterpolationPreviousIndex(2, 2), 0U);
  EXPECT_THROW(detail::boundsInterpolationPreviousIndex(0, 0), std::invalid_argument);
  EXPECT_THROW(detail::boundsInterpolationPreviousIndex(0, 1), std::invalid_argument);
}

TEST(MptBoundsInterpolation, RearCircleUsesFrontBoundsIndependentOfFarTail)
{
  std::vector<geometry_msgs::Point> points(4);
  for (size_t i = 0; i < points.size(); ++i) points[i].x = static_cast<double>(i);
  autoware::interpolation::SplineInterpolationPoints2d spline(points);
  std::vector<Bounds> bounds{{-1.0, 1.0}, {-1.0, 1.0}, {5.0, 6.0}, {7.0, 8.0}};
  const auto interpolate = [&](const double offset) {
    const auto offset_index = spline.getOffsetIndex(0, offset);
    const auto previous = detail::boundsInterpolationPreviousIndex(offset_index, points.size());
    const double a = spline.getAccumulatedLength(previous);
    const double b = spline.getAccumulatedLength(previous + 1);
    return Bounds::lerp(bounds[previous], bounds[previous + 1],
      std::clamp((offset-a)/(b-a), 0.0, 1.0));
  };
  EXPECT_EQ(spline.getOffsetIndex(0, -0.15), 0U);
  EXPECT_EQ(spline.getOffsetIndex(0, 0.0), 1U);
  for (const double offset : {-0.15, 0.0, 0.15, 1.0}) {
    EXPECT_DOUBLE_EQ(interpolate(offset).lower_bound, -1.0);
    EXPECT_DOUBLE_EQ(interpolate(offset).upper_bound, 1.0);
  }
  EXPECT_DOUBLE_EQ(interpolate(5.0).upper_bound, 8.0);
  bounds[2] = Bounds{-20.0, -10.0};
  bounds[3] = Bounds{-40.0, -30.0};
  EXPECT_DOUBLE_EQ(interpolate(-0.15).lower_bound, -1.0);
  EXPECT_DOUBLE_EQ(interpolate(-0.15).upper_bound, 1.0);
}

TEST(MptConstraintMatrix, CopiesEveryKinematicStateColumn)
{
  constexpr size_t reference_count = 2;
  constexpr size_t state_count = 6;
  constexpr size_t variable_count = 10;

  Eigen::SparseMatrix<double> source(reference_count, state_count);
  source.insert(0, 0) = 1.0;
  source.insert(0, 1) = 2.0;
  source.insert(1, 3) = 3.0;
  source.insert(1, 4) = 4.0;

  Eigen::MatrixXd destination = Eigen::MatrixXd::Zero(reference_count, variable_count);
  detail::assignHardConstraintStateBlock(
    destination, source, reference_count, state_count);

  EXPECT_DOUBLE_EQ(destination(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(destination(0, 1), 2.0);
  EXPECT_DOUBLE_EQ(destination(1, 3), 3.0);
  EXPECT_DOUBLE_EQ(destination(1, 4), 4.0);
  EXPECT_DOUBLE_EQ(destination(0, state_count), 0.0);
}

TEST(MptConstraintMatrix, RejectsTruncatedStateBlock)
{
  constexpr size_t reference_count = 2;
  constexpr size_t state_count = 6;

  Eigen::SparseMatrix<double> truncated_source(reference_count, reference_count);
  Eigen::MatrixXd destination = Eigen::MatrixXd::Zero(reference_count, state_count);

  EXPECT_THROW(
    detail::assignHardConstraintStateBlock(
      destination, truncated_source, reference_count, state_count),
    std::invalid_argument);
}

TEST(CircleFootprint, ChecksFullCircleAgainstBounds)
{
  geometry_msgs::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 5.0;
  pose.orientation.w = 1.0;
  const std::vector<geometry_msgs::Point> left = [] {
    std::vector<geometry_msgs::Point> points(2);
    points[0].x = -2.0;
    points[1].x = -2.0;
    points[1].y = 10.0;
    return points;
  }();
  const std::vector<geometry_msgs::Point> right = [] {
    std::vector<geometry_msgs::Point> points(2);
    points[0].x = 2.0;
    points[1].x = 2.0;
    points[1].y = 10.0;
    return points;
  }();
  EXPECT_FALSE(geometry_utils::isOutsideDrivableAreaFromCircleFootprint(
    pose, left, right, {0.5}, {0.0}));
  pose.position.x = 1.6;
  EXPECT_TRUE(geometry_utils::isOutsideDrivableAreaFromCircleFootprint(
    pose, left, right, {0.5}, {0.0}));
  pose.position.x = 0.0;
  EXPECT_TRUE(geometry_utils::isOutsideDrivableAreaFromCircleFootprint(
    pose, left, right, {0.5, 0.5}, {0.0}));
}

TEST(CircleFootprint, ChecksPerpendicularClearanceAtDiagonalBound)
{
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  std::vector<geometry_msgs::Point> left(2);
  std::vector<geometry_msgs::Point> right(2);
  left[0].x = -5.0;
  left[0].y = 0.0;
  left[1].x = 5.0;
  left[1].y = 10.0;
  right[0].x = -5.0;
  right[0].y = -6.3;
  right[1].x = 5.0;
  right[1].y = 3.7;
  // At the origin the right bound is 1.3 m away along the y-axis, but only
  // 1.3/sqrt(2) m away in the perpendicular direction.
  EXPECT_TRUE(geometry_utils::isOutsideDrivableAreaFromCircleFootprint(
    pose, left, right, {1.0}, {0.0}));
  EXPECT_FALSE(geometry_utils::isOutsideDrivableAreaFromCircleFootprint(
    pose, left, right, {0.8}, {0.0}));
}
}  // namespace autoware::path_optimizer

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

namespace autoware::path_optimizer {
TEST(MptConstraintMatrix, SameDimensionsDoNotMakeCscValueUpdatesEquivalent) {
  using autoware::osqp_interface::OSQPInterface;
  using autoware::osqp_interface::calCSCMatrix;
  const Eigen::MatrixXd P = Eigen::MatrixXd::Identity(2,2);
  const Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2,2);
  Eigen::MatrixXd swapped(2,2); swapped << 0,1,1,0;
  const std::vector<double> q{-2,-4}, lower{0,0}, upper{1,3};
  OSQPInterface previous(P,A,q,lower,upper,1e-6);
  auto first = previous.optimize();
  ASSERT_EQ(first.solution_status,1);
  EXPECT_NEAR(first.primal_solution[0],1,1e-3);
  EXPECT_NEAR(first.primal_solution[1],3,1e-3);
  const auto before = calCSCMatrix(A), after = calCSCMatrix(swapped);
  ASSERT_EQ(before.m_vals,after.m_vals);
  ASSERT_EQ(before.m_col_idxs,after.m_col_idxs);
  ASSERT_NE(before.m_row_idxs,after.m_row_idxs);
  // A value-only update cannot express these new row indices. Rebuild the QP;
  // the swapped constraints require x[1] <= 1 instead of x[0] <= 1.
  OSQPInterface rebuilt(P,swapped,q,lower,upper,1e-6);
  const auto second = rebuilt.optimize();
  ASSERT_EQ(second.solution_status,1);
  EXPECT_NEAR(second.primal_solution[0],2,1e-3);
  EXPECT_NEAR(second.primal_solution[1],1,1e-3);
}
}
