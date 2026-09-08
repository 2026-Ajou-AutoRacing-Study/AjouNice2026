#include <autoware/behavior_velocity_planner_common/utilization/path_utilization.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <gtest/gtest.h>

#include <vector>

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

namespace
{
autoware_planning_msgs::Path makePath(const std::vector<double> & xs, const std::vector<double> & velocities)
{
  autoware_planning_msgs::Path path;
  for (size_t i = 0; i < xs.size(); ++i) {
    autoware_planning_msgs::PathPoint point;
    point.pose.position.x = xs.at(i);
    point.longitudinal_velocity_mps = velocities.at(i);
    path.points.push_back(point);
  }
  return path;
}
}  // namespace

TEST(PathUtilization, FilterLitterPathPoint)
{
  const auto filtered = autoware::behavior_velocity_planner::filterLitterPathPoint(
    makePath({0.0, 1.0, 1.001, 2.0, 3.0}, {5.0, 3.5, 3.5, 3.0, 2.5}));
  ASSERT_EQ(4U, filtered.points.size());
  EXPECT_DOUBLE_EQ(0.0, filtered.points.at(0).pose.position.x);
  EXPECT_DOUBLE_EQ(1.0, filtered.points.at(1).pose.position.x);
  EXPECT_DOUBLE_EQ(2.0, filtered.points.at(2).pose.position.x);
  EXPECT_DOUBLE_EQ(3.0, filtered.points.at(3).pose.position.x);
  EXPECT_DOUBLE_EQ(5.0, filtered.points.at(0).longitudinal_velocity_mps);
  EXPECT_DOUBLE_EQ(3.5, filtered.points.at(1).longitudinal_velocity_mps);
  EXPECT_DOUBLE_EQ(3.0, filtered.points.at(2).longitudinal_velocity_mps);
  EXPECT_DOUBLE_EQ(2.5, filtered.points.at(3).longitudinal_velocity_mps);
}

TEST(PathUtilization, InterpolatePathPreservesStopPositions)
{
  const std::vector<std::vector<double>> xs{
    {0.0, 1.0, 2.0, 3.0}, {0.0, 1.0, 2.1, 3.0}, {0.0, 1.0, 2.001, 3.0},
    {0.0, 1.0, 1.999, 2.0, 2.001, 3.0}, {0.0, 1.0, 1.999, 2.0, 2.001, 3.0},
    {0.0, 1.0, 1.999, 3.0}, {0.0, 0.1, 0.2, 0.3, 0.4}, {0.0, 0.1, 0.2, 0.3, 0.4}};
  const std::vector<std::vector<double>> velocities{
    {5.5, 5.5, 0.0, 0.0}, {5.5, 5.5, 0.0, 0.0}, {5.5, 5.5, 0.0, 0.0},
    {5.5, 5.5, 5.555, 5.5, 0.0, 0.0}, {5.5, 5.5, 5.555, 0.0, 0.0, 0.0},
    {5.5, 5.5, 0.0, 0.0}, {5.5, 5.5, 0.0, 0.0, 0.0}, {5.5, 5.5, 5.5, 5.5, 0.0}};
  const std::vector<double> expected_stop_s{2.0, 2.1, 2.001, 2.001, 2.0, 1.999, 0.2, 0.4};

  for (size_t i = 0U; i < xs.size(); ++i) {
    SCOPED_TRACE(i);
    const auto interpolated = autoware::behavior_velocity_planner::interpolatePath(
      makePath(xs.at(i), velocities.at(i)), 5.0, 1.0);
    const auto zero_velocity_index = autoware::motion_utils::searchZeroVelocityIndex(interpolated.points);
    ASSERT_TRUE(zero_velocity_index.has_value());
    EXPECT_DOUBLE_EQ(
      expected_stop_s.at(i), autoware::motion_utils::calcSignedArcLength(
        interpolated.points, 0U, zero_velocity_index.value()));
  }
}

TEST(PathUtilization, FilterStopPathPoint)
{
  const auto filtered = autoware::behavior_velocity_planner::filterStopPathPoint(
    makePath({0.0, 1.0, 2.0, 3.0, 4.0}, {5.0, 4.0, 0.0, 2.0, 3.0}));
  ASSERT_EQ(5U, filtered.points.size());
  EXPECT_DOUBLE_EQ(5.0, filtered.points.at(0).longitudinal_velocity_mps);
  EXPECT_DOUBLE_EQ(4.0, filtered.points.at(1).longitudinal_velocity_mps);
  EXPECT_DOUBLE_EQ(0.0, filtered.points.at(2).longitudinal_velocity_mps);
  EXPECT_DOUBLE_EQ(0.0, filtered.points.at(3).longitudinal_velocity_mps);
  EXPECT_DOUBLE_EQ(0.0, filtered.points.at(4).longitudinal_velocity_mps);
}
