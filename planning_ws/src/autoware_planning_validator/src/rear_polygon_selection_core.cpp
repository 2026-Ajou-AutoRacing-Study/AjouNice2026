#include "autoware/planning_validator/rear_polygon_selection_core.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <cmath>

namespace autoware::planning_validator
{
namespace
{
using Point2d = boost::geometry::model::d2::point_xy<double>;
using Polygon2d = boost::geometry::model::polygon<Point2d>;

std::pair<Point2d, double> smallest_enclosing_circle(const RearDetectionPolygon & polygon)
{
  constexpr double eps = 1e-5;
  Point2d center(0.0, 0.0);
  double radius_squared = 0.0;
  const auto cross = [](const Point2d & p1, const Point2d & p2) {
    return boost::geometry::get<0>(p1) * boost::geometry::get<1>(p2) -
      boost::geometry::get<1>(p1) * boost::geometry::get<0>(p2);
  };
  const auto subtract = [](const Point2d & lhs, const Point2d & rhs) {
    return Point2d(
      boost::geometry::get<0>(lhs) - boost::geometry::get<0>(rhs),
      boost::geometry::get<1>(lhs) - boost::geometry::get<1>(rhs));
  };
  const auto squared_norm = [](const Point2d & point) {
    return boost::geometry::get<0>(point) * boost::geometry::get<0>(point) +
      boost::geometry::get<1>(point) * boost::geometry::get<1>(point);
  };
  const auto make_circle_2 = [&center, &radius_squared, &subtract, &squared_norm, eps](
                               const Point2d & p1, const Point2d & p2) {
    center = Point2d(
      0.5 * (boost::geometry::get<0>(p1) + boost::geometry::get<0>(p2)),
      0.5 * (boost::geometry::get<1>(p1) + boost::geometry::get<1>(p2)));
    radius_squared = squared_norm(subtract(center, p1)) + eps;
  };
  const auto make_circle_3 = [&center, &radius_squared, &cross, &subtract, &squared_norm, eps](
                               const Point2d & p1, const Point2d & p2, const Point2d & p3) {
    const double a = squared_norm(subtract(p2, p3));
    const double b = squared_norm(subtract(p3, p1));
    const double c = squared_norm(subtract(p1, p2));
    const double s = cross(subtract(p2, p1), subtract(p3, p1));
    if (std::abs(s) < eps) return;
    center = Point2d(
      (a * (b + c - a) * boost::geometry::get<0>(p1) +
       b * (c + a - b) * boost::geometry::get<0>(p2) +
       c * (a + b - c) * boost::geometry::get<0>(p3)) / (4.0 * s * s),
      (a * (b + c - a) * boost::geometry::get<1>(p1) +
       b * (c + a - b) * boost::geometry::get<1>(p2) +
       c * (a + b - c) * boost::geometry::get<1>(p3)) / (4.0 * s * s));
    radius_squared = squared_norm(subtract(center, p1)) + eps;
  };
  const auto in_circle = [&center, &radius_squared, &subtract, &squared_norm](const Point2d & point) {
    return squared_norm(subtract(center, point)) <= radius_squared;
  };

  std::vector<Point2d> points;
  points.reserve(polygon.size());
  for (const auto & point : polygon) points.emplace_back(point.x, point.y);
  for (size_t i = 1; i < points.size(); ++i) {
    const auto & p1 = points[i];
    if (in_circle(p1)) continue;
    make_circle_2(points[0], p1);
    for (size_t j = 0; j < i; ++j) {
      const auto & p2 = points[j];
      if (in_circle(p2)) continue;
      make_circle_2(p1, p2);
      for (size_t k = 0; k < j; ++k) {
        const auto & p3 = points[k];
        if (!in_circle(p3)) make_circle_3(p1, p2, p3);
      }
    }
  }
  return {center, radius_squared};
}

Polygon2d as_boost_polygon(const RearDetectionPolygon & polygon)
{
  Polygon2d converted;
  for (const auto & point : polygon) converted.outer().emplace_back(point.x, point.y);
  if (!converted.outer().empty()) converted.outer().push_back(converted.outer().front());
  return converted;
}
}  // namespace

RearPointCloud::Ptr select_rear_points_in_detection_polygons(
  const std::vector<RearDetectionPolygon> & polygons, const RearPointCloud & points)
{
  auto selected = boost::make_shared<RearPointCloud>();
  for (const auto & polygon : polygons) {
    if (polygon.size() < 3U) continue;
    const auto circle = smallest_enclosing_circle(polygon);
    const auto boost_polygon = as_boost_polygon(polygon);
    for (const auto & point : points) {
      const double dx = boost::geometry::get<0>(circle.first) - point.x;
      const double dy = boost::geometry::get<1>(circle.first) - point.y;
      if (dx * dx + dy * dy > circle.second) continue;
      if (boost::geometry::within(Point2d(point.x, point.y), boost_polygon)) selected->push_back(point);
    }
  }
  return selected;
}
}  // namespace autoware::planning_validator
