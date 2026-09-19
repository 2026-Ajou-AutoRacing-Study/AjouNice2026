#include <autoware/motion_velocity_planner/motion_velocity_planner_core.hpp>

#include <gtest/gtest.h>

TEST(MotionVelocityPlannerCore, SequentialModuleResultsNeverReleaseStop)
{
  std::vector<autoware_planning_msgs::TrajectoryPoint> input(3);
  for (size_t index = 0; index < input.size(); ++index) {
    input.at(index).pose.position.x = static_cast<double>(index) * 5.0;
    input.at(index).pose.orientation.w = 1.0;
    input.at(index).longitudinal_velocity_mps = 10.0F;
  }
  autoware::motion_velocity_planner::VelocityPlanningResult stop_result;
  geometry_msgs::Point stop; stop.x = 5.0;
  stop_result.stop_points.push_back(stop);
  autoware::motion_velocity_planner::VelocityPlanningResult slowdown_result;
  geometry_msgs::Point from; from.x = 4.0;
  geometry_msgs::Point to; to.x = 10.0;
  slowdown_result.slowdown_intervals.emplace_back(from, to, 3.0);
  const auto output = autoware::motion_velocity_planner::MotionVelocityPlannerCore().applyResults(input, {stop_result, slowdown_result});
  ASSERT_GT(output.size(), input.size());
  for (const auto & point : output) if (point.pose.position.x >= 5.0) EXPECT_FLOAT_EQ(point.longitudinal_velocity_mps, 0.0F);
}
TEST(MotionVelocityPlannerCore, AppliesStopResultBetweenExistingSamples)
{
  std::vector<autoware_planning_msgs::TrajectoryPoint> input(3);
  for (size_t i = 0; i < input.size(); ++i) { input.at(i).pose.position.x = i * 5.0; input.at(i).pose.orientation.w = 1.0; input.at(i).longitudinal_velocity_mps = 8.0F; }
  autoware::motion_velocity_planner::VelocityPlanningResult result;
  geometry_msgs::Point stop; stop.x=7.0; result.stop_points.push_back(stop);
  const auto output = autoware::motion_velocity_planner::MotionVelocityPlannerCore().applyResults(input, {result});
  for (const auto & point : output) if (point.pose.position.x >= 7.0) EXPECT_FLOAT_EQ(point.longitudinal_velocity_mps, 0.0F);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
