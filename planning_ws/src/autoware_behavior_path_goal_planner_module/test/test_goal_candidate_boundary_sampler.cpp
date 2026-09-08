#include <autoware/behavior_path_goal_planner_module/goal_candidate_boundary_sampler.hpp>

#include <gtest/gtest.h>

namespace
{
geometry_msgs::Pose poseAt(const double x)
{
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.orientation.w = 1.0;
  return pose;
}

lanelet::ConstLineString3d line()
{
  lanelet::Points3d points;
  points.emplace_back(1, 0.0, 0.0, 0.0);
  points.emplace_back(2, 10.0, 0.0, 0.0);
  return lanelet::ConstLineString3d(10, points);
}
}  // namespace

TEST(GoalCandidateBoundarySampler, CropsAndResamplesReferenceCenteredRange)
{
  const auto samples = autoware::behavior_path_planner::sampleGoalCandidateBoundary(
    line(), poseAt(5.0), 2.0, 3.0, 1.0);
  ASSERT_EQ(samples.size(), 5U);
  EXPECT_NEAR(samples.front().position.x, 3.0, 1e-9);
  EXPECT_NEAR(samples.back().position.x, 7.0, 1e-9);
  EXPECT_NEAR(samples.at(2).orientation.w, 1.0, 1e-9);
}

TEST(GoalCandidateBoundarySampler, RejectsInvalidSamplingInput)
{
  EXPECT_TRUE(autoware::behavior_path_planner::sampleGoalCandidateBoundary(
    line(), poseAt(5.0), 2.0, 3.0, 0.0).empty());
  lanelet::Points3d one_point;
  one_point.emplace_back(1, 0.0, 0.0, 0.0);
  EXPECT_TRUE(autoware::behavior_path_planner::sampleGoalCandidateBoundary(
    lanelet::ConstLineString3d(11, one_point), poseAt(0.0), 1.0, 1.0, 1.0).empty());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
