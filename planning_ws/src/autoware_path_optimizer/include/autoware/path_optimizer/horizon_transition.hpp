#ifndef AUTOWARE__PATH_OPTIMIZER__HORIZON_TRANSITION_HPP_
#define AUTOWARE__PATH_OPTIMIZER__HORIZON_TRANSITION_HPP_

#include "autoware/path_optimizer/type_alias.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace autoware::path_optimizer
{
// Compare published (extended/resampled) geometry, not the raw MPT state headings.
// A successful shorter QP alone does not make switching its near field continuous.
inline bool isHorizonReturnContinuous(
  const std::vector<TrajectoryPoint> & previous,
  const std::vector<TrajectoryPoint> & candidate,
  const geometry_msgs::Pose & ego, const double forward_length)
{
  if (previous.size() < 2 || candidate.size() < 2) return false;
  const auto distance2 = [](const auto & a, const auto & b) {
    return std::hypot(a.x - b.x, a.y - b.y);
  };
  const auto yaw = [](const auto & q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  };
  const auto wrap = [](const double a) { return std::atan2(std::sin(a), std::cos(a)); };
  const auto nearest = [&](const auto & points) {
    size_t result = 0;
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < points.size(); ++i) {
      const double d = distance2(points[i].pose.position, ego.position);
      if (d < best) { best = d; result = i; }
    }
    return result;
  };
  const size_t nearest_previous = nearest(previous);
  const size_t begin = nearest_previous > 0 ? nearest_previous - 1 : 0;
  size_t end = begin;
  double length = 0.0;
  while (end + 1 < previous.size() && length < forward_length + 5.0) {
    length += distance2(previous[end].pose.position, previous[end + 1].pose.position);
    ++end;
  }
  if (begin == end) return false;
  length = 0.0;
  size_t checked = 0;
  for (size_t i = nearest(candidate); i < candidate.size(); ++i) {
    if (checked) length += distance2(candidate[i - 1].pose.position, candidate[i].pose.position);
    if (length > forward_length) break;
    const auto & p = candidate[i].pose;
    double best = std::numeric_limits<double>::infinity();
    double reference_yaw = 0.0;
    for (size_t j = begin; j < end; ++j) {
      const auto & a = previous[j].pose;
      const auto & b = previous[j + 1].pose;
      const double dx = b.position.x - a.position.x;
      const double dy = b.position.y - a.position.y;
      const double denominator = dx * dx + dy * dy;
      if (denominator < 1e-12) continue;
      const double ratio = std::clamp(
        ((p.position.x - a.position.x) * dx + (p.position.y - a.position.y) * dy) /
        denominator, 0.0, 1.0);
      const double distance = std::hypot(
        p.position.x - a.position.x - ratio * dx,
        p.position.y - a.position.y - ratio * dy);
      if (distance < best) {
        best = distance;
        reference_yaw = yaw(a.orientation) + ratio * wrap(yaw(b.orientation) - yaw(a.orientation));
      }
    }
    // 2 cm / 0.5 degree bounds apply only to an optional horizon reduction.
    // They never relax the QP's road, obstacle, steering or docking constraints.
    const double heading_change = wrap(yaw(p.orientation) - reference_yaw);
    if (!std::isfinite(best) || !std::isfinite(heading_change) || best > 0.02 ||
        std::abs(heading_change) > 0.008726646259971648) {
      return false;
    }
    ++checked;
  }
  return checked >= 2;
}
}  // namespace autoware::path_optimizer

#endif
