// Copyright 2026 AjouNice2026
// Licensed under the Apache License, Version 2.0.
#include <autoware/path_smoother/elastic_band.hpp>
#include <gtest/gtest.h>

namespace autoware::path_smoother
{
namespace
{
std::vector<TrajectoryPoint> path(const double lateral = 0.0)
{
  std::vector<TrajectoryPoint> result;
  for (int i = 0; i <= 200; ++i) {
    TrajectoryPoint point;
    point.pose.position.x = i * 0.5;
    point.pose.position.y = lateral;
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 16.0;
    result.push_back(point);
  }
  return result;
}

class EBHistoryReset : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node = std::make_unique<rclcpp::Node>(ros::NodeHandle("~"));
    smoother = makeSmoother();
    ego.position.x = 10.0;
    ego.orientation.w = 1.0;
  }

  std::unique_ptr<EBPathSmoother> makeSmoother()
  {
    return std::make_unique<EBPathSmoother>(
      node.get(), false, EgoNearestParam(node.get()), CommonParam(node.get()),
      std::make_shared<TimeKeeper>());
  }

  std::unique_ptr<rclcpp::Node> node;
  std::unique_ptr<EBPathSmoother> smoother;
  geometry_msgs::Pose ego;
};

TEST_F(EBHistoryReset, FreshPathHasNoHistoryToDiscard)
{
  EXPECT_FALSE(smoother->requiresHistoryReset(path(1.55), ego));
}

TEST_F(EBHistoryReset, ContinuousPathAndForwardCroppingKeepHistory)
{
  ASSERT_FALSE(smoother->smoothTrajectory(path(), ego).empty());
  EXPECT_FALSE(smoother->requiresHistoryReset(path(), ego));
  ego.position.x += 0.7;
  EXPECT_FALSE(smoother->requiresHistoryReset(path(), ego));
}

TEST_F(EBHistoryReset, SmallLateralUpdateKeepsHistory)
{
  ASSERT_FALSE(smoother->smoothTrajectory(path(), ego).empty());
  EXPECT_FALSE(smoother->requiresHistoryReset(path(0.02), ego));
}

TEST_F(EBHistoryReset, PathSwitchBelowGenericTwoMetreThresholdResets)
{
  ASSERT_FALSE(smoother->smoothTrajectory(path(), ego).empty());
  EXPECT_TRUE(smoother->requiresHistoryReset(path(1.55), ego));
  smoother->resetPreviousData();
  EXPECT_FALSE(smoother->requiresHistoryReset(path(1.55), ego));
}

TEST_F(EBHistoryReset, DirectSmoothingAfterSwitchMatchesFreshSmoothing)
{
  ASSERT_FALSE(smoother->smoothTrajectory(path(), ego).empty());
  const auto switched = smoother->smoothTrajectory(path(1.55), ego);
  auto fresh = makeSmoother();
  const auto expected = fresh->smoothTrajectory(path(1.55), ego);
  ASSERT_EQ(switched.size(), expected.size());
  ASSERT_FALSE(switched.empty());
  for (size_t i = 0; i < switched.size(); ++i) {
    EXPECT_NEAR(switched[i].pose.position.x, expected[i].pose.position.x, 1e-6);
    EXPECT_NEAR(switched[i].pose.position.y, expected[i].pose.position.y, 1e-6);
  }
  EXPECT_FALSE(smoother->requiresHistoryReset(path(1.55), ego));
}
}  // namespace
}  // namespace autoware::path_smoother

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test_eb_history_reset");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
