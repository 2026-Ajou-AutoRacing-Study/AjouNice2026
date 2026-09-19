#include <autoware/motion_velocity_planner_common/collision_checker.hpp>

#include <boost/geometry/algorithms/correct.hpp>
#include <gtest/gtest.h>

namespace
{
using autoware::motion_velocity_planner::CollisionChecker;
using autoware_utils_geometry::MultiPolygon2d;
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;

Polygon2d rectangle(const double min_x, const double min_y, const double max_x, const double max_y)
{
  Polygon2d value;
  value.outer() = {{min_x, min_y}, {max_x, min_y}, {max_x, max_y}, {min_x, max_y}, {min_x, min_y}};
  boost::geometry::correct(value);
  return value;
}
}  // namespace

TEST(MotionVelocityCollisionChecker, ReturnsOnlyIntersectingTrajectoryFootprints)
{
  MultiPolygon2d footprints{rectangle(0.0, -1.0, 2.0, 1.0), rectangle(4.0, -1.0, 6.0, 1.0)};
  CollisionChecker checker(footprints);

  const auto collisions = checker.getCollisions(rectangle(4.5, -0.5, 5.5, 0.5));

  ASSERT_EQ(checker.trajectorySize(), 2U);
  ASSERT_EQ(collisions.size(), 1U);
  EXPECT_EQ(collisions.front().trajectory_index, 1U);
  EXPECT_FALSE(collisions.front().collision_points.empty());
}

TEST(MotionVelocityCollisionChecker, UsesCoveredByForPointQueriesOnBoost171)
{
  MultiPolygon2d footprints{rectangle(0.0, -1.0, 2.0, 1.0)};
  CollisionChecker checker(footprints);

  const auto inside = checker.getCollisions(Point2d{1.0, 0.0});
  const auto outside = checker.getCollisions(Point2d{3.0, 0.0});

  ASSERT_EQ(inside.size(), 1U);
  ASSERT_EQ(inside.front().collision_points.size(), 1U);
  EXPECT_TRUE(outside.empty());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
