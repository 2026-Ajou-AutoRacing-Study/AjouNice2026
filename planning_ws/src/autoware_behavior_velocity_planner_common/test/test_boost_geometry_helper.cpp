#include <autoware/behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>

#include <gtest/gtest.h>

namespace autoware::behavior_velocity_planner
{
TEST(BoostGeometryHelper, PreservesPolygonOrderingAndScaleOrigin)
{
  LineString2d left{{0.0, 0.0}, {0.0, 2.0}};
  LineString2d right{{2.0, 0.0}, {2.0, 2.0}};
  const auto polygon = lines2polygon(left, right);
  ASSERT_EQ(polygon.outer().size(), 5U);
  EXPECT_DOUBLE_EQ(polygon.outer().front().x(), polygon.outer().back().x());
  EXPECT_DOUBLE_EQ(polygon.outer().front().y(), polygon.outer().back().y());

  geometry_msgs::Point origin;
  origin.x = 1.0;
  origin.y = 1.0;
  const auto scaled = upScalePolygon(origin, polygon, 2.0);
  const auto message = toGeomPoly(scaled);
  ASSERT_EQ(message.points.size(), 5U);
  EXPECT_FLOAT_EQ(message.points.at(0).x, -1.0F);
  EXPECT_FLOAT_EQ(message.points.at(0).y, -1.0F);
}
}  // namespace autoware::behavior_velocity_planner
