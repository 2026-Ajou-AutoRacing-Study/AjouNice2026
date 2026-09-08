// Copyright 2023 Tier IV, Inc.
// Licensed under the Apache License, Version 2.0

#include <autoware/behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>

#include <boost/geometry/algorithms/correct.hpp>

namespace autoware::behavior_velocity_planner
{
Polygon2d lines2polygon(const LineString2d & left_line, const LineString2d & right_line)
{
  Polygon2d polygon;
  polygon.outer().push_back(left_line.front());
  for (const auto & point : right_line) polygon.outer().push_back(point);
  for (auto point = left_line.rbegin(); point != left_line.rend(); ++point) {
    polygon.outer().push_back(*point);
  }
  bg::correct(polygon);
  return polygon;
}

Polygon2d upScalePolygon(
  const geometry_msgs::Point & position, const Polygon2d & polygon, const double scale)
{
  Polygon2d transformed;
  for (const auto & point : polygon.outer()) {
    transformed.outer().emplace_back(
      (point.x() - position.x) * scale + position.x,
      (point.y() - position.y) * scale + position.y);
  }
  return transformed;
}

geometry_msgs::Polygon toGeomPoly(const Polygon2d & polygon)
{
  geometry_msgs::Polygon message;
  for (const auto & point : polygon.outer()) {
    geometry_msgs::Point32 output;
    output.x = static_cast<float>(point.x());
    output.y = static_cast<float>(point.y());
    message.points.push_back(output);
  }
  return message;
}
}  // namespace autoware::behavior_velocity_planner
