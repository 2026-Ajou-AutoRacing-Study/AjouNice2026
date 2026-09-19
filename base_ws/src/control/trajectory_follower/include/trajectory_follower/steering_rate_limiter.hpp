#pragma once

#include <algorithm>
#include <cmath>

namespace trajectory_follower
{

inline double applySteeringDeadband(
  const double target, const double previous, const double deadband_rad)
{
  return std::abs(target - previous) < deadband_rad ? previous : target;
}

inline double limitSteeringRate(
  const double target, const double previous, const double maximum_rate_rad_s,
  const double dt)
{
  const double maximum_delta = maximum_rate_rad_s * dt;
  const double requested_delta = target - previous;
  return previous + std::max(-maximum_delta, std::min(maximum_delta, requested_delta));
}

}  // namespace trajectory_follower
