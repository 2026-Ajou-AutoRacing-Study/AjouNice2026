#include <autoware/motion_velocity_planner_common/utils.hpp>
#include <autoware/motion_velocity_planner_common/collision_checker.hpp>

#include <gtest/gtest.h>

namespace
{
std::vector<autoware_planning_msgs::TrajectoryPoint> straightTrajectory()
{
  std::vector<autoware_planning_msgs::TrajectoryPoint> points(3);
  for (size_t i = 0; i < points.size(); ++i) {
    points.at(i).pose.position.x = static_cast<double>(i) * 5.0;
    points.at(i).pose.orientation.w = 1.0;
    points.at(i).longitudinal_velocity_mps = 10.0F;
  }
  return points;
}
}

TEST(VelocityPlanningResult, AppliesOrderedSlowdownThenTerminalStop)
{
  auto trajectory = straightTrajectory();
  autoware::motion_velocity_planner::VelocityPlanningResult result;
  geometry_msgs::Point from; from.x = 2.0;
  geometry_msgs::Point to; to.x = 7.0;
  result.slowdown_intervals.emplace_back(from, to, 3.0);
  geometry_msgs::Point stop; stop.x = 8.0;
  result.stop_points.push_back(stop);
  autoware::motion_velocity_planner::utils::applyPlanningResult(trajectory, result);
  ASSERT_GE(trajectory.size(), 5U);
  bool saw_slowdown = false;
  bool saw_terminal_stop = false;
  for (const auto & point : trajectory) {
    if (point.pose.position.x >= 2.0 && point.pose.position.x <= 7.0 && point.longitudinal_velocity_mps <= 3.0F) saw_slowdown = true;
    if (point.pose.position.x >= 8.0 && point.longitudinal_velocity_mps == 0.0F) saw_terminal_stop = true;
  }
  EXPECT_TRUE(saw_slowdown);
  EXPECT_TRUE(saw_terminal_stop);
}

TEST(VelocityPlanningResult, SlowdownNeverReleasesAnExistingStop)
{
  auto trajectory = straightTrajectory();
  autoware::motion_velocity_planner::VelocityPlanningResult result;
  geometry_msgs::Point stop; stop.x = 5.0;
  result.stop_points.push_back(stop);
  geometry_msgs::Point from; from.x = 4.0;
  geometry_msgs::Point to; to.x = 10.0;
  result.slowdown_intervals.emplace_back(from, to, 3.0);
  autoware::motion_velocity_planner::utils::applyPlanningResult(trajectory, result);
  for (const auto & point : trajectory) {
    if (point.pose.position.x >= 5.0) EXPECT_FLOAT_EQ(point.longitudinal_velocity_mps, 0.0F);
  }
}

TEST(CollisionChecker, ReturnsOnlyIntersectingTrajectoryFootprints)
{
  using autoware_utils_geometry::Point2d;
  using autoware_utils_geometry::Polygon2d;
  Polygon2d first; first.outer() = {Point2d{0.0, 0.0}, Point2d{2.0, 0.0}, Point2d{2.0, 2.0}, Point2d{0.0, 2.0}, Point2d{0.0, 0.0}};
  Polygon2d second; second.outer() = {Point2d{5.0, 0.0}, Point2d{7.0, 0.0}, Point2d{7.0, 2.0}, Point2d{5.0, 2.0}, Point2d{5.0, 0.0}};
  autoware_utils_geometry::MultiPolygon2d footprints{first, second};
  autoware::motion_velocity_planner::CollisionChecker checker(footprints);
  ASSERT_EQ(checker.trajectorySize(), 2U);
  const auto hits = checker.getCollisions(Point2d{6.0, 1.0});
  ASSERT_EQ(hits.size(), 1U);
  EXPECT_EQ(hits.front().trajectory_index, 1U);
  EXPECT_FALSE(hits.front().collision_points.empty());
  EXPECT_TRUE(checker.getCollisions(Point2d{3.0, 1.0}).empty());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
