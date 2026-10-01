#pragma once

#include <algorithm>
#include <cmath>

namespace trajectory_follower
{

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
