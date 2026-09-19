#include <autoware/behavior_velocity_planner_common/utilization/debug.hpp>

#include <gtest/gtest.h>

namespace autoware::behavior_velocity_planner::debug
{
TEST(DebugMarkerHelpers, RetainMarkerShapeNamespaceAndLifetime)
{
  geometry_msgs::Polygon polygon;
  geometry_msgs::Point32 first;
  first.x = 1.0F;
  polygon.points.push_back(first);
  geometry_msgs::Point32 second;
  second.y = 2.0F;
  polygon.points.push_back(second);
  const auto polygon_markers = createPolygonMarkerArray(
    polygon, "polygon", 17, ros::Time(3.0), 0.1, 0.2, 0.3, 0.4, 0.5, 0.6);
  ASSERT_EQ(polygon_markers.markers.size(), 1U);
  const auto & line = polygon_markers.markers.front();
  EXPECT_EQ(line.type, visualization_msgs::Marker::LINE_STRIP);
  EXPECT_EQ(line.ns, "polygon");
  EXPECT_EQ(line.id, 17);
  ASSERT_EQ(line.points.size(), 3U);
  EXPECT_DOUBLE_EQ(line.points.front().x, line.points.back().x);
  EXPECT_DOUBLE_EQ(line.points.front().y, line.points.back().y);
  EXPECT_DOUBLE_EQ(line.lifetime.toSec(), 0.3);

  std::vector<geometry_msgs::Point> points(2);
  points.at(1).x = 8.0;
  const auto point_markers = createPointsMarkerArray(
    points, "points", 2, ros::Time(4.0), 1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
  ASSERT_EQ(point_markers.markers.size(), 2U);
  EXPECT_EQ(point_markers.markers.at(0).type, visualization_msgs::Marker::SPHERE);
  EXPECT_DOUBLE_EQ(point_markers.markers.at(1).pose.position.x, 8.0);
  EXPECT_NE(point_markers.markers.at(0).id, point_markers.markers.at(1).id);
}
}  // namespace autoware::behavior_velocity_planner::debug
