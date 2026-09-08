#include <autoware/behavior_velocity_crosswalk_module/util.hpp>
#include <gtest/gtest.h>

namespace autoware::behavior_velocity_planner
{
namespace
{
experimental::Trajectory makeTrajectory()
{
  std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> points;
  for (const double x : {0.0, 5.0, 10.0}) {
    autoware_internal_planning_msgs::PathPointWithLaneId point;
    point.point.pose.position.x = x; point.point.pose.orientation.w = 1.0;
    point.point.longitudinal_velocity_mps = 5.0F; points.push_back(point);
  }
  return experimental::Trajectory::Builder{}.build(points).value();
}
TEST(CrosswalkUtil, FindsContinuousEntryAndExit)
{
  const lanelet::BasicPolygon2d polygon{{4.0, -1.0}, {6.0, -1.0}, {6.0, 1.0}, {4.0, 1.0}};
  geometry_msgs::Point ego;
  const auto endpoints = getPathEndPointsOnCrosswalk(makeTrajectory(), polygon, ego);
  ASSERT_TRUE(endpoints.has_value());
  EXPECT_NEAR(endpoints->first.x, 4.0, 1e-3);
  EXPECT_NEAR(endpoints->second.x, 6.0, 1e-3);
}
}  // namespace
}  // namespace autoware::behavior_velocity_planner
int main(int argc, char ** argv) { testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS(); }
