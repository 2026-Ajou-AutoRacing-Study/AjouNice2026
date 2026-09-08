#include <autoware/behavior_path_goal_planner_module/goal_candidate_generator_core.hpp>

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

autoware_utils_geometry::LinearRing2d footprint()
{
  return {{0.25, 0.25}, {0.25, -0.25}, {-0.25, -0.25}, {-0.25, 0.25}, {0.25, 0.25}};
}

autoware_utils_geometry::Polygon2d rectangle(const double min_x, const double min_y, const double max_x, const double max_y)
{
  autoware_utils_geometry::Polygon2d polygon;
  polygon.outer() = {{min_x, min_y}, {max_x, min_y}, {max_x, max_y}, {min_x, max_y}, {min_x, min_y}};
  return polygon;
}
}  // namespace

TEST(GoalCandidateGeneratorCore, ExpandsLeftSideCandidatesAndBreaksAtNoParkingArea)
{
  autoware::behavior_path_planner::GoalCandidateAreaConstraints constraints;
  constraints.departure_check_area = rectangle(-5.0, -5.0, 5.0, 5.0);
  constraints.no_parking_areas = {rectangle(-1.0, -1.25, 1.0, -0.75)};
  const std::vector<autoware::behavior_path_planner::GoalCandidateSeed> seeds{
    {poseAt(0.0, 0.0), 7.0}};

  const auto candidates = autoware::behavior_path_planner::generateGoalCandidatesFromSeeds(
    seeds, true, 2.0, 1.0, footprint(), constraints);
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates.front().id, 0U);
  EXPECT_NEAR(candidates.front().goal_pose.position.y, 0.0, 1e-9);
  EXPECT_NEAR(candidates.front().distance_from_original_goal, 7.0, 1e-9);
}

TEST(GoalCandidateGeneratorCore, PreservesRightSideIdsAndFailsClosedForInvalidInterval)
{
  autoware::behavior_path_planner::GoalCandidateAreaConstraints constraints;
  constraints.departure_check_area = rectangle(-5.0, -5.0, 5.0, 5.0);
  const std::vector<autoware::behavior_path_planner::GoalCandidateSeed> seeds{
    {poseAt(0.0, 0.0), 1.0}, {poseAt(1.0, 0.0), 2.0}};
  const auto candidates = autoware::behavior_path_planner::generateGoalCandidatesFromSeeds(
    seeds, false, 1.0, 1.0, footprint(), constraints);
  ASSERT_EQ(candidates.size(), 4U);
  EXPECT_EQ(candidates.at(0).id, 0U);
  EXPECT_EQ(candidates.at(3).id, 3U);
  EXPECT_NEAR(candidates.at(1).goal_pose.position.y, 1.0, 1e-9);
  EXPECT_TRUE(autoware::behavior_path_planner::generateGoalCandidatesFromSeeds(
    seeds, false, 1.0, 0.0, footprint(), constraints).empty());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
