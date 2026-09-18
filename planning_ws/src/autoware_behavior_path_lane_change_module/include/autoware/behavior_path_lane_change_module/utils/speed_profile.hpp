// Spatial LC speed profile: one representation for output and collision prediction.
#ifndef AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__SPEED_PROFILE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__SPEED_PROFILE_HPP_

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner::utils::lane_change
{
inline std::vector<double> make_speed_profile(
  const std::vector<double> & s, const std::vector<double> & limits,
  const double initial_speed, const double acceleration, const double deceleration)
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
    v[i] = std::min(v[i], std::sqrt(v[i - 1] * v[i - 1] + 2 * acceleration * (s[i] - s[i - 1])));
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
