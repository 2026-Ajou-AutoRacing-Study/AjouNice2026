#ifndef AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__SPEED_REFRESH_HPP_
#define AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__SPEED_REFRESH_HPP_

#include "autoware/behavior_path_lane_change_module/utils/speed_profile.hpp"
#include <autoware_internal_planning_msgs/PathWithLaneId.h>

namespace autoware::behavior_path_planner::utils::lane_change
{
struct RefreshLcMetrics
{
  double duration;
  double terminal_velocity;
  double maximum_velocity;
};

inline RefreshLcMetrics refresh_lc_metrics(
  const autoware_internal_planning_msgs::PathWithLaneId & path,
  const size_t begin, const size_t lc_end)
{
  if (begin >= lc_end || lc_end >= path.points.size()) {
    throw std::logic_error("Invalid refresh LC metadata interval");
  }
  std::vector<double> s{0.0}, v;
  for (size_t i = begin; i <= lc_end; ++i) {
    if (i > begin) {
      const auto & a = path.points[i - 1].point.pose.position;
      const auto & b = path.points[i].point.pose.position;
      s.push_back(s.back() + std::hypot(b.x-a.x, b.y-a.y));
    }
    v.push_back(path.points[i].point.longitudinal_velocity_mps);
  }
  return {speed_profile_duration(s, v), v.back(), *std::max_element(v.begin(), v.end())};
}

template<class Predictions>
bool complete_refresh_predictions(const Predictions & predictions)
{
  if (predictions.size() != 2) return false;
  for (const auto & prediction : predictions) {
    if (prediction.size() < 2 || prediction.front().time != 0.0 ||
        !std::isfinite(prediction.back().time) || prediction.back().time <= 0.0) return false;
    for (size_t i = 1; i < prediction.size(); ++i) {
      if (!std::isfinite(prediction[i].time) ||
          prediction[i].time <= prediction[i - 1].time) return false;
    }
  }
  return true;
}

// The safety callback sees the FINAL assembled output, not a pre-stop draft.
// No refresh state is committed on fallback, callback rejection or exception.
template<class Candidate, class SafetyCheck>
bool commit_final_speed_refresh(
  Candidate & approved, autoware_internal_planning_msgs::PathWithLaneId & output,
  Candidate proposal, const bool fallback, SafetyCheck safety_check)
{
  if (fallback || proposal.path.points.size() != output.points.size()) return false;
  for (size_t i = 0; i < output.points.size(); ++i) {
    const auto & before = output.points[i];
    const auto & after = proposal.path.points[i];
    if (before.point.pose != after.point.pose || before.lane_ids != after.lane_ids ||
        (before.point.longitudinal_velocity_mps <= 0.0 &&
         before.point.longitudinal_velocity_mps != after.point.longitudinal_velocity_mps)) return false;
  }
  if (!safety_check(proposal)) return false;
  auto final_output = proposal.path;
  approved = std::move(proposal);
  output = std::move(final_output);
  return true;
}

// Caps must come from the road/upstream path, NOT the previously approved LC
// velocities. Only the requested longitudinal interval is changed; geometry,
// lane IDs and every existing zero-speed point remain untouched.
inline autoware_internal_planning_msgs::PathWithLaneId refresh_path_speed(
  const autoware_internal_planning_msgs::PathWithLaneId & original,
  const size_t begin, const size_t end, const std::vector<double> & external_caps,
  const double initial_speed, const double acceleration, const double deceleration,
  const autoware::motion_utils::AccelerationLimits & acceleration_limits = {})
{
  if (begin >= end || end >= original.points.size() || external_caps.size() != end - begin + 1) {
    throw std::logic_error("Invalid approved LC refresh interval");
  }
  auto caps = external_caps;
  std::vector<double> distances{0.0};
  for (size_t i = begin; i <= end; ++i) {
    if (original.points[i].point.longitudinal_velocity_mps <= 0.0) caps[i - begin] = 0.0;
    if (i > begin) {
      const auto & a = original.points[i - 1].point.pose.position;
      const auto & b = original.points[i].point.pose.position;
      const double ds = std::hypot(b.x - a.x, b.y - a.y);
      if (ds < 1e-6) throw std::logic_error("Repeated approved LC point");
      distances.push_back(distances.back() + ds);
    }
  }
  // Do not create a speed discontinuity where the unchanged suffix begins.
  caps.back() = std::min(caps.back(),
    static_cast<double>(original.points[end].point.longitudinal_velocity_mps));
  const auto velocities = make_speed_profile(
    distances, caps, initial_speed, acceleration, deceleration, acceleration_limits);
  auto result = original;
  for (size_t i = begin; i <= end; ++i) {
    result.points[i].point.longitudinal_velocity_mps = velocities[i - begin];
  }
  return result;
}
}  // namespace autoware::behavior_path_planner::utils::lane_change
#endif
