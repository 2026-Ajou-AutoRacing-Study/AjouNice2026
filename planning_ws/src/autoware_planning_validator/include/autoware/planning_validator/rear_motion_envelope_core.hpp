#ifndef AUTOWARE_PLANNING_VALIDATOR_REAR_MOTION_ENVELOPE_CORE_HPP_
#define AUTOWARE_PLANNING_VALIDATOR_REAR_MOTION_ENVELOPE_CORE_HPP_

#include <autoware_planning_msgs/Trajectory.h>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include <optional>

namespace autoware::planning_validator
{
// Source-faithful input boundary for rear_collision_checker::calc_lookahead_line
// and calc_predicted_stop_line. No wall-clock or manufactured pose is allowed.
struct RearMotionEnvelopeInput
{
  const autoware_planning_msgs::Trajectory & trajectory;
  const nav_msgs::Odometry & current_kinematics;
  const geometry_msgs::AccelWithCovarianceStamped & current_acceleration;
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info;
};

struct RearMotionEnvelopeConstraints
{
  double lookahead_time{4.0};
  double max_deceleration{-4.0};
  double max_positive_jerk{5.0};
  double max_negative_jerk{-5.0};
};

struct RearMotionEnvelope
{
  geometry_msgs::Pose lookahead_left;
  geometry_msgs::Pose lookahead_right;
  geometry_msgs::Pose predicted_stop_left;
  geometry_msgs::Pose predicted_stop_right;
  double lookahead_distance{0.0};
  double predicted_stop_distance{0.0};
};

// Returns nullopt if either source line cannot be placed on the input
// trajectory. A caller must then suppress activation rather than infer safety.
std::optional<RearMotionEnvelope> compute_rear_motion_envelope(
  const RearMotionEnvelopeInput & input, const RearMotionEnvelopeConstraints & constraints);
}  // namespace autoware::planning_validator

#endif
