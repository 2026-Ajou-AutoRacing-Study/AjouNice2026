// Copyright 2026 AjouNice2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0

#include "autoware/path_optimizer/mpt_optimizer.hpp"
#include "autoware/path_optimizer/utils/geometry_utils.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include <stdexcept>

namespace autoware::path_optimizer
{
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
