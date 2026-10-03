#pragma once

#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner::detail {
// Negative longitudinal acceleration makes distance knots nonuniform in time.
// Interpolating the lateral values against those distances creates a nonzero
// endpoint tangent. Recover their original times before evaluating the SAME
// lateral cubic. Positive/zero-acceleration coordinates are left untouched.
inline bool reparameterize_negative_shift(
  double velocity, double acceleration, std::vector<double> & bases,
  std::vector<double> & query)
{
  if (!std::isfinite(velocity) || !std::isfinite(acceleration))
    throw std::logic_error("Nonfinite shift motion.");
  const double v = std::abs(velocity);
  if (acceleration >= -1e-4 || v < 1e-5) return false;
  const auto convert = [&](const std::vector<double> & stations, bool strict) {
    std::vector<double> result;
    result.reserve(stations.size());
    for (const double s : stations) {
      if (!std::isfinite(s) || s < 0.0)
        throw std::logic_error("Invalid negative shift station.");
      const double squared = v * v + 2.0 * acceleration * s;
      if (!std::isfinite(squared) || squared < 0.0)
        throw std::logic_error("Decelerating shift would stop before its end.");
      const double time = 2.0 * s / (v + std::sqrt(squared));
      if (!std::isfinite(time) || (!result.empty() &&
          (strict ? time <= result.back() : time < result.back())))
        throw std::logic_error("Nonmonotone negative shift time.");
      result.push_back(time);
    }
    return result;
  };
  auto converted_bases = convert(bases, true);
  auto converted_query = convert(query, false);
  bases = std::move(converted_bases);
  query = std::move(converted_query);
  return true;
}
}  // namespace autoware::behavior_path_planner::detail
