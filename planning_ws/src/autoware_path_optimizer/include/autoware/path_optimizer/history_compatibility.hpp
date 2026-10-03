#ifndef AUTOWARE__PATH_OPTIMIZER__HISTORY_COMPATIBILITY_HPP_
#define AUTOWARE__PATH_OPTIMIZER__HISTORY_COMPATIBILITY_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

namespace autoware::path_optimizer::detail
{
// Checks only the geometry introduced by reattaching a historical prefix.
// Position(point) returns an object with x/y fields (no ROS/ReferencePoint dependency).
// The caller must keep the unmodified fresh reference until this check succeeds.
// A false result rejects history reuse, NOT current road/vehicle safety constraints.
template<class Points, class Position>
bool historyPrefixCompatible(
  const Points & candidate, const Points & fresh, const double curvature_limit,
  const double resample_spacing, Position position)
{
  if (candidate.size() < 3 || fresh.size() < 3 ||
      !std::isfinite(curvature_limit) || curvature_limit <= 0.0 ||
      !std::isfinite(resample_spacing) || resample_spacing <= 0.0) return false;

  const auto distance = [](const auto & a, const auto & b) {
    return std::hypot(a.x - b.x, a.y - b.y);
  };
  const auto curvature = [&](const auto & points, const size_t i) {
    const auto a = position(points[i - 1]);
    const auto b = position(points[i]);
    const auto c = position(points[i + 1]);
    const double denominator = distance(a, b) * distance(b, c) * distance(a, c);
    if (!std::isfinite(denominator) || denominator < 1e-10) {
      return std::numeric_limits<double>::infinity();
    }
    return 2.0 * std::abs((b.x-a.x)*(c.y-a.y) - (b.y-a.y)*(c.x-a.x)) / denominator;
  };
  // Compare against the same physical part of the fresh polyline, not its index.
  const auto project = [&](const auto & p) {
    double best = std::numeric_limits<double>::infinity();
    size_t segment = 0;
    for (size_t j = 0; j + 1 < fresh.size(); ++j) {
      const auto a = position(fresh[j]);
      const auto b = position(fresh[j + 1]);
      const double dx = b.x-a.x, dy = b.y-a.y;
      const double squared = dx*dx + dy*dy;
      if (!std::isfinite(squared) || squared < 1e-12) continue;
      const double r = std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/squared, 0.0, 1.0);
      const double d = std::hypot(p.x-a.x-r*dx, p.y-a.y-r*dy);
      if (d < best) { best = d; segment = j; }
    }
    return std::make_pair(best, segment);
  };

  // Three consecutive points agreeing with the fresh line and its forward
  // tangent establish rejoining; inspect two further points for spline overshoot.
  // 5% of the resampling step is a join-identification tolerance, not a road margin.
  const double join_tolerance = std::max(1e-4, 0.05 * resample_spacing);
  size_t consecutive = 0;
  size_t inspect_end = candidate.size() - 2;
  for (size_t i = 0; i + 1 < candidate.size(); ++i) {
    const auto p = position(candidate[i]);
    const auto q = position(candidate[i + 1]);
    const auto projection = project(p);
    const auto a = position(fresh[projection.second]);
    const auto b = position(fresh[projection.second + 1]);
    const double cross = (q.x-p.x)*(b.y-a.y) - (q.y-p.y)*(b.x-a.x);
    const double dot = (q.x-p.x)*(b.x-a.x) + (q.y-p.y)*(b.y-a.y);
    const double angle = std::abs(std::atan2(cross, dot));
    const bool joined = projection.first <= join_tolerance && dot > 0.0 &&
      angle <= curvature_limit * resample_spacing + 1e-6;
    consecutive = joined ? consecutive + 1 : 0;
    if (consecutive >= 3) {
      inspect_end = std::min(candidate.size() - 2, i + 2);
      break;
    }
  }

  for (size_t i = 1; i <= inspect_end; ++i) {
    const double current_curvature = curvature(candidate, i);
    if (current_curvature <= curvature_limit + 1e-6) continue;
    const auto projection = project(position(candidate[i]));
    if (!std::isfinite(projection.first)) return false;
    const size_t low = projection.second > 1 ? projection.second - 1 : 1;
    const size_t high = std::min(fresh.size() - 2, projection.second + 2);
    double fresh_curvature = 0.0;
    for (size_t j = low; j <= high; ++j) {
      fresh_curvature = std::max(fresh_curvature, curvature(fresh, j));
    }
    // Attribute only newly introduced infeasible curvature to history. An
    // already-invalid fresh reference remains the responsibility of existing gates.
    if (fresh_curvature <= curvature_limit + 1e-6) return false;
  }
  return true;
}
}  // namespace autoware::path_optimizer::detail
#endif  // AUTOWARE__PATH_OPTIMIZER__HISTORY_COMPATIBILITY_HPP_
