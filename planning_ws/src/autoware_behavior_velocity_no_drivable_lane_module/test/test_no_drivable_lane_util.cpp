#include "util.hpp"

#include <gtest/gtest.h>

#include <lanelet2_core/primitives/Polygon.h>

#include <vector>

namespace autoware::behavior_velocity_planner::no_drivable_lane
{
namespace
{
using Trajectory = autoware::experimental::trajectory::Trajectory<
  autoware_internal_planning_msgs::PathPointWithLaneId>;

Trajectory makeStraightTrajectory()
{
  std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> points;
  for (const double x : {0.0, 5.0, 10.0}) {
    autoware_internal_planning_msgs::PathPointWithLaneId point;
    point.point.pose.position.x = x;
    point.point.pose.orientation.w = 1.0;
    point.point.longitudinal_velocity_mps = 5.0F;
    points.push_back(point);
  }
  return Trajectory::Builder{}.build(points).value();
}

lanelet::BasicPolygon2d makeCrossingPolygon()
{
  return lanelet::BasicPolygon2d{{lanelet::BasicPoint2d{4.0, -1.0},
                                  lanelet::BasicPoint2d{6.0, -1.0},
                                  lanelet::BasicPoint2d{6.0, 1.0},
                                  lanelet::BasicPoint2d{4.0, 1.0}}};
}

TEST(NoDrivableLaneUtil, ClassifiesEntryAndExitOnRos1Trajectory)
{
  const auto intersection = getPathIntersectionWithNoDrivableLanePolygon(
    makeStraightTrajectory(), makeCrossingPolygon(), 2U);

  EXPECT_FALSE(intersection.is_path_inside_of_polygon);
  EXPECT_FALSE(intersection.is_first_path_point_inside_polygon);
  ASSERT_TRUE(intersection.first_intersection_s.has_value());
  ASSERT_TRUE(intersection.second_intersection_s.has_value());
  EXPECT_NEAR(*intersection.first_intersection_s, 4.0, 1e-3);
  EXPECT_NEAR(*intersection.second_intersection_s, 6.0, 1e-3);
}
}  // namespace
}  // namespace autoware::behavior_velocity_planner::no_drivable_lane

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
