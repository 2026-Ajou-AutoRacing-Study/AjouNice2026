// Copyright 2026 AjouNice contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef AUTOWARE_ROUTE_HANDLER__ROUTE_LANELET_TIE_HPP_
#define AUTOWARE_ROUTE_HANDLER__ROUTE_LANELET_TIE_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <tuple>

namespace autoware::route_handler::detail
{
// Secondary ordering ONLY for exactly equal polygon distance. Project onto finite
// centerline segments, not sampled vertices; ties do not depend on R-tree order.
struct RouteLaneletTie
{
  bool valid{false};
  double heading_error{std::numeric_limits<double>::infinity()};
  double squared_distance{std::numeric_limits<double>::infinity()};
  std::int64_t id{};
};

template <class Points>
RouteLaneletTie routeLaneletTie(
  const Points & points, double x, double y, double yaw, std::int64_t id)
{
  RouteLaneletTie result;
  result.id = id;
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) return result;
  for (std::size_t i = 1; i < points.size(); ++i) {
    const double ax = points[i - 1].x(), ay = points[i - 1].y();
    const double bx = points[i].x(), by = points[i].y();
    if (!std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(bx) ||
        !std::isfinite(by)) return RouteLaneletTie{false, INFINITY, INFINITY, id};
    const double dx = bx - ax, dy = by - ay, length2 = dx * dx + dy * dy;
    if (!(length2 > 0.0) || !std::isfinite(length2)) continue;
    const double t = std::clamp(((x - ax) * dx + (y - ay) * dy) / length2, 0.0, 1.0);
    const double ex = x - (ax + t * dx), ey = y - (ay + t * dy);
    const double distance = ex * ex + ey * ey;
    const double angle = std::atan2(dy, dx) - yaw;
    const double error = std::abs(std::atan2(std::sin(angle), std::cos(angle)));
    if (!std::isfinite(distance) || !std::isfinite(error)) continue;
    if (distance < result.squared_distance ||
        (distance == result.squared_distance && error < result.heading_error)) {
      result = RouteLaneletTie{true, error, distance, id};
    }
  }
  return result;
}

inline bool routeLaneletTieLess(const RouteLaneletTie & a, const RouteLaneletTie & b)
{
  // Distinguish forward-compatible from opposite-direction centerlines first.
  // Within the same direction class, geometric proximity wins over small yaw
  // noise: exact heading priority can select a distant parallel lane.
  const double half_pi = std::acos(-1.0) * 0.5;
  return std::make_tuple(
    !a.valid, a.heading_error > half_pi, a.squared_distance, a.heading_error, a.id) <
    std::make_tuple(
    !b.valid, b.heading_error > half_pi, b.squared_distance, b.heading_error, b.id);
}
}  // namespace autoware::route_handler::detail
#endif
