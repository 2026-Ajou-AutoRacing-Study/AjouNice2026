// Copyright 2026 The Autoware Contributors
// Licensed under the Apache License, Version 2.0.
#ifndef AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__EXECUTION_PROGRESS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__EXECUTION_PROGRESS_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace autoware::behavior_path_planner::utils::lane_change
{
struct ExecutionProgressPoint
{
  double s;
  double lateral;
  double time;
  double yaw;
};

// Compare lateral maneuver phase at the same road progress, not Euclidean
// distance divided by longitudinal speed. This is a progress monitor; collision
// and whole-body road containment remain independent hard checks.
inline double execution_recovery_time(
  const std::vector<ExecutionProgressPoint> & profile, double s, double lateral,
  double yaw, double speed, double lateral_tolerance, double max_acc, double max_jerk)
{
  const double unavailable = std::numeric_limits<double>::infinity();
  if (profile.size() < 2 || !std::isfinite(s) || !std::isfinite(lateral) ||
      !std::isfinite(yaw) || !std::isfinite(speed) || !std::isfinite(max_acc) ||
      !std::isfinite(max_jerk) || max_acc <= 0.0 || max_jerk <= 0.0) return unavailable;
  const auto interp = [](double a, double b, double r) { return a + r * (b - a); };
  double at_time = profile.front().time;
  for (size_t i = 1; i < profile.size(); ++i) {
    if (s > profile[i].s && i + 1 < profile.size()) continue;
    const auto & a = profile[i - 1]; const auto & b = profile[i];
    const double ratio = std::clamp((s - a.s) / std::max(1e-6, b.s - a.s), 0.0, 1.0);
    at_time = interp(a.time, b.time, ratio);
    break;
  }
  double required = unavailable;
  for (size_t i = 1; i < profile.size(); ++i) {
    const auto & a = profile[i - 1]; const auto & b = profile[i];
    // Find the closest lateral phase on each segment, including constant-lateral
    // prepare/finish sections. A genuine phase delay then has no lateral jump.
    const double ratio = std::abs(b.lateral - a.lateral) > 1e-6
      ? std::clamp((lateral - a.lateral) / (b.lateral - a.lateral), 0.0, 1.0)
      : std::clamp((s - a.s) / std::max(1e-6, b.s - a.s), 0.0, 1.0);
    const double residual = std::max(
      0.0, std::abs(lateral - interp(a.lateral, b.lateral, ratio)) - lateral_tolerance);
    const double phase_delay = std::abs(at_time - interp(a.time, b.time, ratio));
    const double yaw_delta = std::atan2(std::sin(b.yaw - a.yaw), std::cos(b.yaw - a.yaw));
    const double heading_error = std::atan2(
      std::sin(yaw - a.yaw - ratio * yaw_delta),
      std::cos(yaw - a.yaw - ratio * yaw_delta));
    const double heading_time = std::abs(heading_error) * std::max(1.0, std::abs(speed)) / max_acc;
    // Rest-to-rest quintic correction: peak acceleration <= 6*d/T^2 and
    // peak jerk <= 60*d/T^3. No additional tuning constants are introduced.
    const double correction = std::max(
      std::sqrt(6.0 * residual / max_acc), std::cbrt(60.0 * residual / max_jerk));
    required = std::min(required, std::max(phase_delay + correction, heading_time));
  }
  return required;
}

struct ExecutionRecoveryMonitor
{
  std::optional<double> failure_since;
  bool expired(double now, bool recoverable, double allowance)
  {
    if (recoverable || (failure_since && now < *failure_since)) failure_since.reset();
    if (recoverable) return false;
    if (!failure_since) failure_since = now;
    return now - *failure_since >= allowance;
  }
};
}  // namespace autoware::behavior_path_planner::utils::lane_change
#endif
