// Copyright 2026 AjouNice2026
// Licensed under the Apache License, Version 2.0.
#include <autoware/path_optimizer/replan_checker.hpp>
#include <gtest/gtest.h>

namespace autoware::path_optimizer
{
namespace
{
PlannerData data(const double lateral = 0.0, const int start = 0)
{
  PlannerData result;
  result.ego_pose.position.x = 10.0;
  result.ego_pose.orientation.w = 1.0;
  result.ego_vel = 16.0;
  for (int i = start; i <= 200; ++i) {
    TrajectoryPoint point;
    point.pose.position.x = i * 0.5;
    point.pose.position.y = lateral;
    point.pose.orientation.w = 1.0;
    result.traj_points.push_back(point);
  }
  return result;
}

class MPTHistoryReset : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node = std::make_unique<rclcpp::Node>(ros::NodeHandle("~"));
    checker = std::make_unique<ReplanChecker>(node.get(), EgoNearestParam(node.get()));
    checker->updateData(data(), true, ros::Time(1));
  }
  std::unique_ptr<rclcpp::Node> node;
  std::unique_ptr<ReplanChecker> checker;
};

TEST_F(MPTHistoryReset, ContinuousPathAndForwardCroppingKeepHistory)
{
  EXPECT_FALSE(checker->isResetRequired(data()));
  auto next = data(0.0, 2);
  next.ego_pose.position.x += 1.0;
  EXPECT_FALSE(checker->isResetRequired(next));
}

TEST_F(MPTHistoryReset, SmallLateralUpdateKeepsHistory)
{
  EXPECT_FALSE(checker->isResetRequired(data(0.02)));
}

TEST_F(MPTHistoryReset, SwitchedBackwardAnchorResetsBelowGenericThreshold)
{
  EXPECT_TRUE(checker->isResetRequired(data(1.55)));
}

TEST_F(MPTHistoryReset, ForwardShapeChangeDoesNotDiscardCompatibleHistory)
{
  auto next = data();
  for (auto & point : next.traj_points) {
    if (point.pose.position.x > 30.0) point.pose.position.y = 0.01 * (point.pose.position.x - 30.0);
  }
  EXPECT_FALSE(checker->isResetRequired(next));
}

TEST_F(MPTHistoryReset, MissingPathRequiresReset)
{
  auto next = data();
  next.traj_points.clear();
  EXPECT_TRUE(checker->isResetRequired(next));
}
}  // namespace
}  // namespace autoware::path_optimizer

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test_mpt_history_reset");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
