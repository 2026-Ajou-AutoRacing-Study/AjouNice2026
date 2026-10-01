#pragma once

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace trajectory_follower
{

using AccelerationMap = std::vector<std::pair<double, double>>;

inline bool validAccelerationMap(const AccelerationMap & map)
{
  double previous = -1.0;
  for (const auto & point : map) {
    if (!std::isfinite(point.first) || !std::isfinite(point.second) ||
        point.first < 0.0 || point.first <= previous || point.second <= 0.0) {
      return false;
    }
    previous = point.first;
  }
  return true;
}

// Calibrated full-throttle acceleration. Clamp endpoints; never extrapolate.
inline double accelerationAtSpeed(
  const AccelerationMap & map, const double speed, const double fallback)
{
  if (map.empty() || !std::isfinite(speed) || !validAccelerationMap(map)) {
    return fallback;
  }
  if (speed <= map.front().first) {return map.front().second;}
  for (std::size_t i = 1; i < map.size(); ++i) {
    if (speed <= map[i].first) {
      const double ratio = (speed - map[i - 1].first) / (map[i].first - map[i - 1].first);
      return map[i - 1].second + ratio * (map[i].second - map[i - 1].second);
    }
  }
  return map.back().second;
}

// In the existing brake-entry deadband, taper instead of switching FF off at
// error == 0. At the unchanged brake-entry threshold, FF is exactly zero.
inline double accelerationFeedforwardWeight(const double error, const double brake_entry)
{
  if (!std::isfinite(error) || !std::isfinite(brake_entry) || brake_entry < 0.0) {
    return 0.0;
  }
  if (error >= 0.0) {return 1.0;}
  return brake_entry > 0.0 ? std::max(0.0, 1.0 + error / brake_entry) : 0.0;
}

// Positive acceleration only; brake arbitration stays with the caller.
inline double accelerationFeedforward(
  const bool enabled, const bool acceleration_allowed, const double planned_acceleration,
  const double max_acceleration, const double gain)
{
  if (!enabled || !acceleration_allowed || !std::isfinite(planned_acceleration) ||
      !std::isfinite(max_acceleration) || max_acceleration <= 0.0 ||
      !std::isfinite(gain) || gain < 0.0) {
    return 0.0;
  }
  return std::min(1.0, gain * std::max(0.0, planned_acceleration) / max_acceleration);
}

// Only limit increases; braking and lower requests must take effect immediately.
inline double limitFeedforwardRise(
  const double requested, const double previous, const double rate, const double dt)
{
  if (!std::isfinite(requested) || requested <= 0.0 || !std::isfinite(previous) ||
      !std::isfinite(rate) || rate < 0.0 || !std::isfinite(dt) || dt <= 0.0) {
    return 0.0;
  }
  return rate == 0.0 ? requested :
    std::min(requested, std::max(0.0, previous) + rate * dt);
}

// v^2 is linear in distance for constant acceleration. Never extrapolate and
// never modify a flat, decreasing, stopped or invalid profile.
inline double interpolateIncreasingVelocity(
  const double first, const double second, const double ratio, const double fallback)
{
  if (!std::isfinite(first) || !std::isfinite(second) || !std::isfinite(ratio) ||
      first <= 0.0 || second <= first) {
    return fallback;
  }
  const double t = std::max(0.0, std::min(1.0, ratio));
  return std::hypot(std::sqrt(1.0 - t) * first, std::sqrt(t) * second);
}

struct BrakeHysteresisState
{
  bool active{false};
  double hard_exit_elapsed_s{0.0};
};

inline double resetAccelerationIntegralOnBraking(
  const double error, const double integral, const bool enabled)
{
  if (!enabled) {
    return integral;
  }
  const bool accel_integral_opposes_braking = error < 0.0 && integral > 0.0;
  return accel_integral_opposes_braking ? 0.0 : integral;
}

inline BrakeHysteresisState updateBrakeHysteresis(
  const BrakeHysteresisState & state, const double error,
  const double planned_acceleration, const double dt, const bool enabled,
  const double enter_error, const double exit_error, const double hard_exit_error,
  const double hard_exit_hold_s, const double planned_deceleration_threshold)
{
  if (!enabled) {
    return {};
  }
  if (!state.active) {
    return {error <= -enter_error, 0.0};
  }
  const bool trajectory_is_decelerating =
    planned_acceleration <= -planned_deceleration_threshold;
  if (error >= exit_error && !trajectory_is_decelerating) {
    return {};
  }
  const bool hard_exit_requested = error >= hard_exit_error;
  double hard_exit_elapsed_s = hard_exit_requested
    ? state.hard_exit_elapsed_s + (dt > 0.0 ? dt : 0.0) : 0.0;
  if (hard_exit_requested && hard_exit_elapsed_s >= hard_exit_hold_s) {
    return {};
  }
  return {true, hard_exit_elapsed_s};
}

}  // namespace trajectory_follower
