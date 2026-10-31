// Spatial LC speed profile: one representation for output and collision prediction.
#ifndef AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__SPEED_PROFILE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__SPEED_PROFILE_HPP_

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <limits>
#include <utility>
#include <vector>
#include <autoware/motion_utils/trajectory/acceleration_limits.hpp>

namespace autoware::behavior_path_planner::utils::lane_change
{
// Re-anchor the remaining candidate motion at the measured current state.
// Preparation uses the sampled candidate acceleration; execution retains the
// original spatial acceleration increments. Road speeds remain upper bounds.
// Returns true only when the execution profile needs its existing limits reapplied.
inline bool reanchor_speed_profile(
  std::vector<double> & s, std::vector<double> & v, const double initial_speed,
  const double preparation_acceleration, const double lane_change_start)
{
  if (s.size() != v.size() || s.size() < 2 || !std::isfinite(initial_speed) ||
      initial_speed < 0.0 || !std::isfinite(preparation_acceleration) ||
      !std::isfinite(lane_change_start)) {
    throw std::logic_error("Invalid LC prediction initial state");
  }
  for (size_t i = 0; i < s.size(); ++i) {
    if (!std::isfinite(s[i]) || !std::isfinite(v[i]) || v[i] < 0.0 ||
        (i && s[i] <= s[i - 1])) {
      throw std::logic_error("Invalid remaining LC spatial profile");
    }
  }
  const auto energy_at = [&](const double at) {
    if (at <= s.front()) return v.front() * v.front();
    const auto upper = std::lower_bound(s.begin(), s.end(), at);
    if (upper == s.end()) return v.back() * v.back();
    const size_t i = std::distance(s.begin(), upper);
    const double ratio = (at - s[i - 1]) / (s[i] - s[i - 1]);
    return v[i - 1] * v[i - 1] + ratio * (v[i] * v[i] - v[i - 1] * v[i - 1]);
  };
  const double roundoff = 64.0 * std::numeric_limits<float>::epsilon() *
    std::max({1.0, std::abs(initial_speed), std::abs(v.front())});
  bool preparation_matches = true;
  for (size_t i = 1; i < s.size() && s[i] <= lane_change_start; ++i) {
    const double expected_energy = initial_speed * initial_speed +
      2.0 * preparation_acceleration * (s[i] - s.front());
    preparation_matches = preparation_matches && expected_energy >= 0.0 &&
      std::abs(v[i] - std::sqrt(std::max(0.0, expected_energy))) <= roundoff;
  }
  if (lane_change_start > s.front() && lane_change_start < s.back()) {
    const double expected_energy = initial_speed * initial_speed +
      2.0 * preparation_acceleration * (lane_change_start - s.front());
    preparation_matches = preparation_matches && expected_energy >= 0.0 &&
      std::abs(std::sqrt(std::max(0.0, energy_at(lane_change_start))) -
        std::sqrt(std::max(0.0, expected_energy))) <= roundoff;
  }
  if (preparation_matches && std::abs(v.front() - initial_speed) <= roundoff) {
    // Preserve acceleration throughout the profile, not just its first value.
    const double offset = initial_speed * initial_speed - v.front() * v.front();
    for (auto & speed : v) speed = std::sqrt(std::max(0.0, speed * speed + offset));
    v.front() = initial_speed;
    return false;
  }
  // Keep the preparation/execution boundary explicit even on a sparse path.
  if (lane_change_start > s.front() && lane_change_start < s.back()) {
    const auto pos = std::lower_bound(s.begin(), s.end(), lane_change_start);
    if (*pos != lane_change_start) {
      const size_t index = std::distance(s.begin(), pos);
      const double speed = std::sqrt(std::max(0.0, energy_at(lane_change_start)));
      s.insert(pos, lane_change_start);
      v.insert(v.begin() + index, speed);
    }
  }
  const double preparation_distance = std::max(0.0, lane_change_start - s.front());
  const double boundary_energy = initial_speed * initial_speed +
    2.0 * preparation_acceleration * preparation_distance;
  if (preparation_distance > 0.0 && boundary_energy <= 0.0) {
    throw std::logic_error("LC preparation stops before reaching the execution segment");
  }
  const double original_boundary_energy = energy_at(std::max(s.front(), lane_change_start));
  std::vector<double> anchored;
  anchored.reserve(s.size());
  for (const double at : s) {
    const double energy = at < lane_change_start
      ? initial_speed * initial_speed + 2.0 * preparation_acceleration * (at - s.front())
      : boundary_energy + energy_at(at) - original_boundary_energy;
    anchored.push_back(std::sqrt(std::max(0.0, energy)));
  }
  v = std::move(anchored);
  v.front() = initial_speed;
  return true;
}

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
