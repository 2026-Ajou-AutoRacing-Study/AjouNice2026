#include <autoware/behavior_path_planner_common/utils/utils.hpp>

#include <gtest/gtest.h>
#include <tf2/utils.h>

namespace
{
lanelet::ConstLanelet straightLane()
{
  lanelet::LineString3d left(1, {lanelet::Point3d(2, 0.0, 1.0, 0.0), lanelet::Point3d(3, 10.0, 1.0, 0.0)});
  lanelet::LineString3d right(4, {lanelet::Point3d(5, 0.0, -1.0, 0.0), lanelet::Point3d(6, 10.0, -1.0, 0.0)});
  return lanelet::Lanelet(10, left, right);
}
}  // namespace

TEST(GoalRefinementCore, ProjectsExternalGoalAndRecognizesLaneMembership)
{
  const auto lane = straightLane();
  geometry_msgs::Pose outside{};
  outside.position.x = 4.0;
  outside.position.y = 3.0;

  const auto refined = autoware::behavior_path_planner::utils::refineGoal(outside, lane);
  EXPECT_DOUBLE_EQ(refined.position.x, 4.0);
  EXPECT_DOUBLE_EQ(refined.position.y, 0.0);
  EXPECT_NEAR(tf2::getYaw(refined.orientation), 0.0, 1e-9);
  EXPECT_FALSE(autoware::behavior_path_planner::utils::isInLanelets(outside, {lane}));
  EXPECT_TRUE(autoware::behavior_path_planner::utils::isInLanelets(refined, {lane}));
}
