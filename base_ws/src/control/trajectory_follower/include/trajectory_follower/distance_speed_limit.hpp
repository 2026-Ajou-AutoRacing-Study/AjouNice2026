#pragma once

#include <algorithm>
#include <cmath>

namespace trajectory_follower
{
// Constant-deceleration reachability envelope. Delay is reserved at the larger
// of measured and candidate speed, so a low measured speed cannot hide the
// distance needed while accelerating toward the candidate target.
inline double distanceSpeedLimit(
  double future_speed, double distance, double approach_speed,
  double deceleration, double delay, double margin)
{
  const double available = std::max(0.0, distance - margin - approach_speed * delay);
  return std::sqrt(future_speed * future_speed + 2.0 * deceleration * available);
}

inline double brakingPreviewDistance(
  double approach_speed, double deceleration, double delay, double margin)
{
  return approach_speed * delay + approach_speed * approach_speed / (2.0 * deceleration) + margin;
}
}  // namespace trajectory_follower
