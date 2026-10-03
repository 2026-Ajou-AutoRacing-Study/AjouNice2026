#ifndef AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__PREPARE_SPEED_PROFILE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__PREPARE_SPEED_PROFILE_HPP_
#include "autoware/behavior_path_lane_change_module/utils/speed_profile.hpp"
#include <autoware_internal_planning_msgs/PathWithLaneId.h>

namespace autoware::behavior_path_planner::utils::lane_change
{
// Only the combined output gains an anchor. ShiftLine indices and the shifted
// geometry/shift-length arrays continue to refer to their original path.
inline size_t insert_maneuver_anchor(
  autoware_internal_planning_msgs::PathWithLaneId & path, const size_t segment,
  const geometry_msgs::Pose & anchor)
{
  if (segment + 1 >= path.points.size()) throw std::logic_error("Invalid LC ego anchor segment");
  const auto distance = [&](const auto & p) {
    return std::hypot(p.point.pose.position.x-anchor.position.x,
                      p.point.pose.position.y-anchor.position.y);
  };
  if (distance(path.points[segment]) < 1e-7) return segment;
  if (distance(path.points[segment + 1]) < 1e-7) return segment + 1;
  auto point = path.points[segment];
  point.point.pose = anchor;
  point.point.longitudinal_velocity_mps = std::min(
    point.point.longitudinal_velocity_mps,
    path.points[segment + 1].point.longitudinal_velocity_mps);
  path.points.insert(path.points.begin() + segment + 1, point);
  return segment + 1;
}

inline std::vector<double> make_maneuver_speed_profile(
  const std::vector<double> & s, std::vector<double> caps, const double ego_speed,
  const double prepare_end, const double prepare_acceleration,
  const double changing_acceleration, const double deceleration,
  const autoware::motion_utils::AccelerationLimits & calibration = {})
{
  if (s.size() < 2 || s.size() != caps.size() || s.front() != 0.0 ||
      !std::isfinite(ego_speed + prepare_end + prepare_acceleration +
                     changing_acceleration + deceleration) ||
      ego_speed < 0 || prepare_end < 0 || changing_acceleration <= 0 || deceleration <= 0)
    throw std::logic_error("Invalid full LC speed profile inputs");
  for (size_t i=0;i<s.size();++i) {
    if (!std::isfinite(s[i]+caps[i]) || caps[i]<0 || (i && s[i]<=s[i-1]))
      throw std::logic_error("Invalid full LC speed profile station");
    if (s[i] <= prepare_end + 1e-7) {
      const double nominal = prepare_acceleration < 0
        ? std::sqrt(std::max(0.0,ego_speed*ego_speed+2*prepare_acceleration*s[i]))
        : calibration.reachableVelocity(ego_speed,s[i],prepare_acceleration);
      caps[i]=std::min(caps[i],nominal);
    }
  }
  for (size_t i=caps.size()-1;i>0;--i)
    caps[i-1]=std::min(caps[i-1],std::sqrt(caps[i]*caps[i]+2*deceleration*(s[i]-s[i-1])));
  // Never lower time-zero speed to pretend a stop/road cap is feasible.
  if (ego_speed > caps.front()+1e-5)
    throw std::logic_error("LC prepare speed profile infeasible at measured ego speed");
  caps.front()=ego_speed;
  for (size_t i=1;i<caps.size();++i) {
    const double acceleration = s[i] <= prepare_end+1e-7
      ? std::max(0.0,prepare_acceleration) : changing_acceleration;
    caps[i]=std::min(caps[i],calibration.reachableVelocity(caps[i-1],s[i]-s[i-1],acceleration));
  }
  return caps;
}
}
#endif
