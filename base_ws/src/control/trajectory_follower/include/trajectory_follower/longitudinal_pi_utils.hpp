#pragma once

namespace trajectory_follower
{

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
