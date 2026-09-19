#include <autoware/behavior_path_goal_planner_module/goal_candidate_boundary_core.hpp>

#include <gtest/gtest.h>

namespace
{
geometry_msgs::Pose poseAt(const double x, const double y)
{
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.w = 1.0;
  return pose;
}

lanelet::ConstLineString3d horizontalBoundary(const double y)
{
  lanelet::Points3d points;
  points.emplace_back(1, -10.0, y, 0.0);
  points.emplace_back(2, 10.0, y, 0.0);
  return lanelet::ConstLineString3d(10, points);
}
}  // namespace

TEST(GoalCandidateBoundaryCore, PreservesLeftAndRightMinimumMarginPolicies)
{
  const auto front = poseAt(2.0, 0.0);
  const auto rear = poseAt(-2.0, 0.0);
  const auto boundary = horizontalBoundary(4.0);

  const auto left = autoware::behavior_path_planner::calculateGoalPoseWithMinBoundaryMargin(
    front, rear, boundary, 2.0, 0.5, 1.0, true);
  const auto right = autoware::behavior_path_planner::calculateGoalPoseWithMinBoundaryMargin(
    front, rear, boundary, 2.0, 0.5, 1.0, false);

  ASSERT_TRUE(left);
  ASSERT_TRUE(right);
  EXPECT_NEAR(left->position.x, 0.0, 1e-9);
  EXPECT_NEAR(left->position.y, -1.5, 1e-9);
  EXPECT_NEAR(right->position.x, 0.0, 1e-9);
  EXPECT_NEAR(right->position.y, 5.5, 1e-9);
}

TEST(GoalCandidateBoundaryCore, RejectsBoundaryWithNoLateralIntersection)
{
  lanelet::Points3d points;
  points.emplace_back(1, 4.0, 3.0, 0.0);
  points.emplace_back(2, 8.0, 3.0, 0.0);
  const auto no_intersection = lanelet::ConstLineString3d(11, points);
  EXPECT_FALSE(autoware::behavior_path_planner::calculateGoalPoseWithMinBoundaryMargin(
    poseAt(2.0, 0.0), poseAt(-2.0, 0.0), no_intersection, 2.0, 0.5, 1.0, true));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
