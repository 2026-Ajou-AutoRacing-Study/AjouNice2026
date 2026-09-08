#include <autoware/behavior_path_goal_planner_module/goal_planner_parameters.hpp>

#include <gtest/gtest.h>

TEST(GoalPlannerParameters, PreservesCriticalSourceDefaultsAndNestedContracts)
{
  const autoware::behavior_path_planner::GoalPlannerParameters parameters{};
  EXPECT_EQ(parameters.parking_policy, autoware::behavior_path_planner::ParkingPolicy::LEFT_SIDE);
  EXPECT_DOUBLE_EQ(parameters.check_collision_duration, 1.0);
  EXPECT_FALSE(parameters.enable_freespace_parking);
  EXPECT_TRUE(parameters.object_recognition_collision_check_hard_margins.empty());
  EXPECT_DOUBLE_EQ(parameters.parallel_parking_parameters.pull_out_arc_path_interval, 0.0);
  EXPECT_EQ(parameters.safety_check_params.method, "RSS");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
