#pragma once

#include <algorithm>

namespace trajectory_follower
{

inline double limitSteeringRate(
  const double target, const double previous, const double maximum_rate_rad_s,
  const double dt)
{
  const double maximum_delta = maximum_rate_rad_s * dt;
  const double requested_delta = target - previous;
  return previous + std::max(-maximum_delta, std::min(maximum_delta, requested_delta));
}

}  // namespace trajectory_follower
