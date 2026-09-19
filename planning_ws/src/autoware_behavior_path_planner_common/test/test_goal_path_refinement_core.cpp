#include <autoware/behavior_path_planner_common/utils/utils.hpp>

#include <gtest/gtest.h>

namespace
{
lanelet::ConstLanelet straightLane()
{
  lanelet::LineString3d left(1, {lanelet::Point3d(2, 0.0, 1.0, 0.0), lanelet::Point3d(3, 10.0, 1.0, 0.0)});
  lanelet::LineString3d right(4, {lanelet::Point3d(5, 0.0, -1.0, 0.0), lanelet::Point3d(6, 10.0, -1.0, 0.0)});
  return lanelet::Lanelet(10, left, right);
}

autoware_internal_planning_msgs::PathWithLaneId straightPath()
{
  autoware_internal_planning_msgs::PathWithLaneId path;
  for (int index = 0; index != 6; ++index) {
    autoware_internal_planning_msgs::PathPointWithLaneId point{};
    point.point.pose.position.x = 2.0 * index;
    point.point.pose.orientation.w = 1.0;
    point.point.longitudinal_velocity_mps = 5.0;
    point.lane_ids = {10};
    path.points.push_back(point);
  }
  return path;
}
}  // namespace

TEST(GoalPathRefinementCore, ReconstructsGoalPathWithMappedLaneAndTerminalStop)
{
  const auto lane = straightLane();
  geometry_msgs::Pose goal{};
  goal.position.x = 9.0;
  goal.orientation.w = 1.0;
  const auto output = autoware::behavior_path_planner::utils::refinePathForGoal(
    4.0, 1.57, 0.5, straightPath(), goal, lane.id(),
    [&lane](const int64_t id) { EXPECT_EQ(id, lane.id()); return lane; });
  ASSERT_FALSE(output.points.empty());
  EXPECT_NEAR(output.points.back().point.pose.position.x, 9.0, 1e-6);
  EXPECT_DOUBLE_EQ(output.points.back().point.longitudinal_velocity_mps, 0.0);
  EXPECT_EQ(output.points.back().lane_ids, std::vector<int64_t>({10}));
}
