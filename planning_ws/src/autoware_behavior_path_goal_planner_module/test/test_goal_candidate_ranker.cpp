#include <autoware/behavior_path_goal_planner_module/goal_candidate_ranker.hpp>

#include <gtest/gtest.h>

namespace
{
autoware::behavior_path_planner::GoalCandidate candidate(
  const size_t id, const double longitudinal, const double lateral, const size_t objects)
{
  autoware::behavior_path_planner::GoalCandidate output{};
  output.id = id;
  output.distance_from_original_goal = longitudinal;
  output.lateral_offset = lateral;
  output.num_objects_to_avoid = objects;
  return output;
}
}  // namespace

TEST(GoalCandidateRanker, PreservesSourceLongitudinalTieAndObjectPriority)
{
  using autoware::behavior_path_planner::GoalCandidates;
  GoalCandidates candidates{candidate(1, 2.005, 2.0, 2), candidate(2, 2.0, 1.0, 2), candidate(3, 4.0, 0.0, 0)};
  autoware::behavior_path_planner::rankGoalCandidates(
    &candidates, "minimum_longitudinal_distance", 0.0, true);
  ASSERT_EQ(candidates.size(), 3U);
  EXPECT_EQ(candidates.at(0).id, 3U);
  EXPECT_EQ(candidates.at(1).id, 2U);
  EXPECT_EQ(candidates.at(2).id, 1U);
}

TEST(GoalCandidateRanker, AppliesWeightedDistanceWithoutReorderingUnknownPriority)
{
  using autoware::behavior_path_planner::GoalCandidates;
  GoalCandidates candidates{candidate(1, 2.0, 5.0, 0), candidate(2, 3.0, 0.0, 0)};
  autoware::behavior_path_planner::rankGoalCandidates(
    &candidates, "minimum_weighted_distance", 1.0, false);
  EXPECT_EQ(candidates.at(0).id, 2U);
  autoware::behavior_path_planner::rankGoalCandidates(&candidates, "unrecognized", 1.0, false);
  EXPECT_EQ(candidates.at(0).id, 2U);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
