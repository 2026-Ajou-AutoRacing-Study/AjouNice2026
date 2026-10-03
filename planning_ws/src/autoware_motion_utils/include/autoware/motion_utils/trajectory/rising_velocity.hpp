#pragma once

#include <geometry_msgs/Point.h>
#include <algorithm>
#include <cmath>

namespace autoware::motion_utils
{
// Opt-in transfer of an increasing spatial speed profile, not acceleration
// planning. Constant v^2 slope represents constant longitudinal acceleration.
// Preserve legacy zero-order hold for stops, reverse, decreasing/invalid data.
inline double rising_velocity_at_projection(
  const geometry_msgs::Point & begin, const geometry_msgs::Point & end,
  const geometry_msgs::Point & query, const double v0, const double v1)
{
  const double dx=end.x-begin.x, dy=end.y-begin.y;
  const double norm=dx*dx+dy*dy;
  const double dot=(query.x-begin.x)*dx+(query.y-begin.y)*dy;
  if (!std::isfinite(norm) || !std::isfinite(dot) || norm<1e-12 ||
      !std::isfinite(v0) || !std::isfinite(v1) || v0<=1e-3 || v1<=v0)
    return v0;
  const double fraction=std::clamp(dot/norm,0.0,1.0);
  return std::clamp(std::sqrt((1.0-fraction)*v0*v0+fraction*v1*v1),v0,v1);
}
}  // namespace autoware::motion_utils
