#include "autoware/planning_validator/rear_motion_envelope_core.hpp"

#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/geometry.hpp>

namespace autoware::planning_validator
{
std::optional<RearMotionEnvelope> compute_rear_motion_envelope(
  const RearMotionEnvelopeInput & input, const RearMotionEnvelopeConstraints & constraints)
{
  const auto & points = input.trajectory.points;
  const auto & ego_pose = input.current_kinematics.pose.pose;
  const auto current_velocity = input.current_kinematics.twist.twist.linear.x;
  const auto current_acceleration = input.current_acceleration.accel.accel.linear.x;
  const auto lookahead_distance = current_velocity * constraints.lookahead_time;
  const auto lookahead_pose = autoware::motion_utils::calcLongitudinalOffsetPose(
    points, ego_pose.position,
    input.vehicle_info.max_longitudinal_offset_m + lookahead_distance);
  if (!lookahead_pose) return std::nullopt;

  constexpr double target_velocity = 0.0;
  const auto opt_stop_distance = autoware::motion_utils::calcDecelDistWithJerkAndAccConstraints(
    current_velocity, target_velocity, current_acceleration, constraints.max_deceleration,
    constraints.max_positive_jerk, constraints.max_negative_jerk);
  const auto stop_distance = opt_stop_distance.value_or(0.0);
  const auto stop_pose = autoware::motion_utils::calcLongitudinalOffsetPose(
    points, ego_pose.position, input.vehicle_info.max_longitudinal_offset_m + stop_distance);
  if (!stop_pose) return std::nullopt;

  const auto offset = 0.5 * input.vehicle_info.vehicle_width_m;
  RearMotionEnvelope result;
  result.lookahead_left = autoware_utils_geometry::calc_offset_pose(*lookahead_pose, 0.0, offset, 0.0);
  result.lookahead_right = autoware_utils_geometry::calc_offset_pose(*lookahead_pose, 0.0, -offset, 0.0);
  result.predicted_stop_left = autoware_utils_geometry::calc_offset_pose(*stop_pose, 0.0, offset, 0.0);
  result.predicted_stop_right = autoware_utils_geometry::calc_offset_pose(*stop_pose, 0.0, -offset, 0.0);
  result.lookahead_distance = lookahead_distance;
  result.predicted_stop_distance = stop_distance;
  return result;
}
}  // namespace autoware::planning_validator
