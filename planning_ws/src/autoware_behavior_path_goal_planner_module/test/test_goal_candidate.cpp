#include <autoware/behavior_path_goal_planner_module/goal_candidate.hpp>

#include <gtest/gtest.h>

TEST(GoalCandidate, PreservesSourceValueDefaults)
{
  const autoware::behavior_path_planner::GoalCandidate candidate{};
  EXPECT_DOUBLE_EQ(candidate.goal_pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(candidate.distance_from_original_goal, 0.0);
  EXPECT_DOUBLE_EQ(candidate.lateral_offset, 0.0);
  EXPECT_EQ(candidate.id, 0U);
  EXPECT_TRUE(candidate.is_safe);
  EXPECT_EQ(candidate.num_objects_to_avoid, 0U);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
