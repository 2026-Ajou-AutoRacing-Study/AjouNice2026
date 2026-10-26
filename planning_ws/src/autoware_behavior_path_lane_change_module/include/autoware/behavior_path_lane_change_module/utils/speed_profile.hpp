// Spatial LC speed profile: one representation for output and collision prediction.
#ifndef AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__SPEED_PROFILE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__SPEED_PROFILE_HPP_

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>
#include <autoware/motion_utils/trajectory/acceleration_limits.hpp>

namespace autoware::behavior_path_planner::utils::lane_change
{
inline std::vector<double> make_speed_profile(
  const std::vector<double> & s, const std::vector<double> & limits,
  const double initial_speed, const double acceleration, const double deceleration,
  const autoware::motion_utils::AccelerationLimits & acceleration_limits = {})
{
  if (s.size() != limits.size() || s.empty() || !std::isfinite(initial_speed) ||
      initial_speed < 0 || !std::isfinite(acceleration) || !std::isfinite(deceleration) ||
      acceleration <= 0 || deceleration <= 0) {
    throw std::logic_error("Invalid LC speed profile inputs");
  }
  auto v = limits;
  for (size_t i = 0; i < s.size(); ++i) {
    if (!std::isfinite(s[i]) || !std::isfinite(v[i]) || v[i] < 0 ||
        (i && s[i] < s[i - 1])) throw std::logic_error("Invalid LC speed profile point");
  }
  // Anticipate lower limits, instead of introducing an instantaneous speed drop.
  for (size_t i = v.size() - 1; i > 0; --i) {
    v[i - 1] = std::min(v[i - 1], std::sqrt(v[i] * v[i] + 2 * deceleration * (s[i] - s[i - 1])));
  }
  if (initial_speed > v.front() + 0.1) {
    throw std::logic_error("LC speed limit cannot be reached with allowed deceleration");
  }
  v.front() = std::min(initial_speed, v.front());
  for (size_t i = 1; i < v.size(); ++i) {
    v[i] = std::min(v[i], acceleration_limits.reachableVelocity(
      v[i - 1], s[i] - s[i - 1], acceleration));
  }
  return v;
}

// Forward executable acceleration ramp. Use the same sampled spatial profile
// for publication and safety prediction. A lower cap requiring excessive jerk
// is rejected rather than approved under an instantaneous acceleration jump.
inline std::vector<double> limit_speed_profile_jerk(
  const std::vector<double> & s, const std::vector<double> & upper,
  const double initial_acceleration, const double max_jerk, const double min_jerk,
  const double max_acceleration,
  const autoware::motion_utils::AccelerationLimits & acceleration_limits = {})
{
  if (s.size() != upper.size() || s.empty() || !std::isfinite(initial_acceleration) ||
      !std::isfinite(max_jerk) || !std::isfinite(min_jerk) || max_jerk <= 0 || min_jerk >= 0 || !std::isfinite(max_acceleration) || max_acceleration <= 0) {
    throw std::logic_error("Invalid LC jerk profile inputs");
  }
  auto v = upper;
  double previous_acceleration = initial_acceleration;
  for (size_t i = 1; i < s.size(); ++i) {
    const double ds = s[i]-s[i-1];
    if (ds <= 1e-8) { v[i] = std::min(v[i],v[i-1]); continue; }
    v[i] = std::min(v[i], acceleration_limits.reachableVelocity(v[i-1], ds, max_acceleration));
    // Bisection against the acceleration averaged over this segment.
    const auto feasible = [&](double next) {
      const double dt = 2*ds/std::max(1e-6,v[i-1]+next);
      const double a = (next-v[i-1])/dt;
      return a <= previous_acceleration + 0.5*max_jerk*dt;
    };
    if (!feasible(v[i])) {
      double lo = 0.0, hi = v[i];
      if (!feasible(lo)) throw std::logic_error("LC acceleration ramp cannot reach speed cap");
      for (int j = 0; j < 48; ++j) {
        const double mid = (lo+hi)*0.5;
        if (feasible(mid)) lo = mid; else hi = mid;
      }
      v[i] = lo;
    }
    const double dt = 2*ds/std::max(1e-6,v[i-1]+v[i]);
    const double acceleration = (v[i]-v[i-1])/dt;
    if (acceleration < previous_acceleration + min_jerk*dt - 1e-3) {
      throw std::logic_error("LC braking cap requires excessive jerk");
    }
    previous_acceleration = acceleration;
  }
  return v;
}

inline double speed_profile_duration(
  const std::vector<double> & s, const std::vector<double> & v)
{
  double duration = 0;
  for (size_t i = 1; i < s.size(); ++i) {
    if (s[i] - s[i - 1] < 1e-8) continue;
    if (v[i] + v[i - 1] < 1e-6) throw std::logic_error("Stopped LC speed profile");
    duration += 2 * (s[i] - s[i - 1]) / (v[i] + v[i - 1]);
  }
  return duration;
}

// An observed output may stop before completing the maneuver. Its occupancy
// remains at the stop pose; it is not an invalid candidate and never teleports
// to the end. Candidate duration validation above intentionally stays strict.
inline double observed_speed_profile_duration(
  const std::vector<double> & s, const std::vector<double> & v,
  const double stationary_horizon)
{
  if (s.size() != v.size() || s.size() < 2 ||
      !std::isfinite(stationary_horizon) || stationary_horizon <= 0) {
    throw std::logic_error("Invalid observed LC speed profile");
  }
  double duration = 0.0;
  for (size_t i = 1; i < s.size(); ++i) {
    if (!std::isfinite(s[i]) || !std::isfinite(s[i-1]) ||
        !std::isfinite(v[i]) || !std::isfinite(v[i-1]) || v[i] < 0 || v[i-1] < 0) {
      throw std::logic_error("Nonfinite observed LC speed profile");
    }
    const double ds = s[i] - s[i-1];
    if (ds < 1e-8) continue;
    if (v[i]+v[i-1] < 1e-6) return std::max(duration, stationary_horizon);
    duration += 2.0*ds/(v[i]+v[i-1]);
  }
  return duration;
}

// Exact integration of the piecewise constant acceleration implied by v(s)^2.
inline std::pair<double, double> sample_speed_profile(
  const std::vector<double> & s, const std::vector<double> & v, double time)
{
  for (size_t i = 1; i < s.size(); ++i) {
    const double ds = s[i] - s[i - 1];
    if (ds < 1e-8) continue;
    if (v[i] + v[i - 1] < 1e-6) return {s[i - 1], 0};
    const double dt = 2 * ds / (v[i] + v[i - 1]);
    if (time <= dt) {
      const double a = (v[i] - v[i - 1]) / dt;
      return {s[i - 1] + v[i - 1] * time + 0.5 * a * time * time,
              v[i - 1] + a * time};
    }
    time -= dt;
  }
  return {s.back(), v.back()};
}
}  // namespace autoware::behavior_path_planner::utils::lane_change
#endif
