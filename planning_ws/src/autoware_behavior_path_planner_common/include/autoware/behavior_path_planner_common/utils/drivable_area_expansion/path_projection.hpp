#pragma once

#include <autoware/behavior_path_planner_common/utils/drivable_area_expansion/types.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace autoware::behavior_path_planner::drivable_area_expansion
{
inline double point_distance(const Point2d & a, const Point2d & b)
{
  return std::hypot(a.x() - b.x(), a.y() - b.y());
}
inline PointDistance point_to_segment_projection(const Point2d & p, const Point2d & p1, const Point2d & p2)
{
  const double dx = p2.x() - p1.x(), dy = p2.y() - p1.y();
  const double squared = dx * dx + dy * dy;
  const double ratio = squared == 0.0 ? 0.0 : std::clamp(((p.x() - p1.x()) * dx + (p.y() - p1.y()) * dy) / squared, 0.0, 1.0);
  const Point2d projected{p1.x() + ratio * dx, p1.y() + ratio * dy};
  return {projected, point_distance(p, projected)};
}
inline PointDistance point_to_line_projection(const Point2d & p, const Point2d & p1, const Point2d & p2)
{
  const double dx = p2.x() - p1.x(), dy = p2.y() - p1.y(), squared = dx * dx + dy * dy;
  const double ratio = squared == 0.0 ? 0.0 : ((p.x() - p1.x()) * dx + (p.y() - p1.y()) * dy) / squared;
  const Point2d projected{p1.x() + ratio * dx, p1.y() + ratio * dy};
  return {projected, point_distance(p, projected)};
}
inline Projection point_to_linestring_projection(const Point2d & p, const LineString2d & ls)
{
  Projection closest; closest.distance = std::numeric_limits<double>::max(); double arc = 0.0;
  for (size_t i = 0; i + 1 < ls.size(); ++i) {
    const auto projection = point_to_segment_projection(p, ls[i], ls[i + 1]);
    if (projection.distance < closest.distance) closest = {projection.point, projection.distance, arc + point_distance(ls[i], projection.point)};
    arc += point_distance(ls[i], ls[i + 1]);
  }
  return closest;
}
inline Point2d normal_at_distance(const Point2d & p1, const Point2d & p2, const double distance)
{
  const double dx = p2.x() - p1.x(), dy = p2.y() - p1.y(), length = std::hypot(dx, dy);
  if (length == 0.0) return p1;
  return {p1.x() - dy / length * distance, p1.y() + dx / length * distance};
}
inline Point2d lerp_point(const Point2d & a, const Point2d & b, const double ratio)
{
  return {a.x() + (b.x() - a.x()) * ratio, a.y() + (b.y() - a.y()) * ratio};
}
}  // namespace autoware::behavior_path_planner::drivable_area_expansion
