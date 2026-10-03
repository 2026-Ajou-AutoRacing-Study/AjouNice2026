#pragma once
#include <autoware_utils/geometry/geometry.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace autoware::behavior_path_planner::utils::lane_change
{
// One physical contract for candidate generation and approved-path refresh.
// Do not substitute a different curvature baseline only after approval.
template<class Points>
double lateral_speed_cap(const Points & points,const size_t i,const double lateral_limit)
{
  if(!std::isfinite(lateral_limit)||lateral_limit<=0||i>=points.size())
    throw std::logic_error("Invalid LC lateral speed cap inputs");
  if(i==0||i+1>=points.size())return std::numeric_limits<double>::infinity();
  const double k=std::abs(autoware_utils::calc_curvature(
    points[i-1].point.pose.position,points[i].point.pose.position,
    points[i+1].point.pose.position));
  if(!std::isfinite(k))throw std::logic_error("Invalid LC curvature");
  return k>1e-8 ? std::sqrt(lateral_limit/k) : std::numeric_limits<double>::infinity();
}
}
