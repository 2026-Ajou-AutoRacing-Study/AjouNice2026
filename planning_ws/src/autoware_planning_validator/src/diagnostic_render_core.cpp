#include "autoware/planning_validator/diagnostic_render_core.hpp"

#include <string>

namespace autoware::planning_validator
{
namespace
{
diagnostic_msgs::DiagnosticStatus make_status(
  const std::string & name, const bool is_ok, const std::string & invalid_message,
  const autoware_planning_validator::PlanningValidatorStatus & validation_status,
  const DiagnosticRenderParameters & parameters, const bool trajectory_provider)
{
  diagnostic_msgs::DiagnosticStatus result;
  result.name = name;
  if (is_ok) {
    result.level = diagnostic_msgs::DiagnosticStatus::OK;
    result.message = "validated.";
    return result;
  }
  if (validation_status.invalid_count < parameters.error_count_threshold) {
    result.level = diagnostic_msgs::DiagnosticStatus::WARN;
    result.message = invalid_message + " (invalid count is less than error threshold: " +
      std::to_string(validation_status.invalid_count) + " < " +
      std::to_string(parameters.error_count_threshold) + ")";
    return result;
  }
  if (trajectory_provider && parameters.trajectory_override_all_errors) {
    result.level = diagnostic_msgs::DiagnosticStatus::WARN;
    result.message = invalid_message + " (error diag was overriden internally)";
    return result;
  }
  result.level = diagnostic_msgs::DiagnosticStatus::ERROR;
  result.message = invalid_message;
  return result;
}
}  // namespace

std::vector<diagnostic_msgs::DiagnosticStatus> render_source_shaped_diagnostics(
  const autoware_planning_validator::PlanningValidatorStatus & status,
  const DiagnosticRenderParameters & parameters)
{
  using Entry = std::tuple<const char *, bool, const char *>;
  const std::vector<Entry> trajectory_entries{
    {"trajectory_validation_size", status.is_valid_size, "invalid trajectory size is found"},
    {"trajectory_validation_finite", status.is_valid_finite_value, "infinite value is found"},
    {"trajectory_validation_interval", status.is_valid_interval, "points interval is too large"},
    {"trajectory_validation_relative_angle", status.is_valid_relative_angle, "relative angle is too large"},
    {"trajectory_validation_curvature", status.is_valid_curvature, "curvature is too large"},
    {"trajectory_validation_lateral_acceleration", status.is_valid_lateral_acc, "lateral acceleration is too large"},
    {"trajectory_validation_acceleration", status.is_valid_longitudinal_max_acc, "acceleration is too large"},
    {"trajectory_validation_deceleration", status.is_valid_longitudinal_min_acc, "deceleration is too large"},
    {"trajectory_validation_steering", status.is_valid_steering, "steering angle is too large"},
    {"trajectory_validation_steering_rate", status.is_valid_steering_rate, "steering rate is too large"},
    {"trajectory_validation_velocity_deviation", status.is_valid_velocity_deviation, "velocity deviation is too large"},
    {"trajectory_validation_distance_deviation", status.is_valid_distance_deviation, "distance deviation is too large"},
    {"trajectory_validation_longitudinal_distance_deviation", status.is_valid_longitudinal_distance_deviation, "longitudinal distance deviation is too large"},
    {"trajectory_validation_yaw_deviation", status.is_valid_yaw_deviation, "difference between vehicle yaw and closest trajectory yaw is too large"},
    {"trajectory_validation_forward_trajectory_length", status.is_valid_forward_trajectory_length, "trajectory length is too short"},
    {"trajectory_validation_trajectory_shift", status.is_valid_trajectory_shift, "detected sudden shift in trajectory"},
  };
  std::vector<diagnostic_msgs::DiagnosticStatus> result;
  result.reserve(19);
  for (const auto & [name, is_ok, message] : trajectory_entries) {
    result.push_back(make_status(name, is_ok, message, status, parameters, true));
  }
  result.push_back(make_status("trajectory_validation_latency", status.is_valid_latency,
    "latency is larger than expected value", status, parameters, false));
  result.push_back(make_status("intersection_validation_collision_check", status.is_valid_intersection_collision_check,
    "risk of collision at intersection turn", status, parameters, false));
  result.push_back(make_status("rear_collision_check", status.is_valid_rear_collision_check,
    "obstacle detected behind the vehicle", status, parameters, false));
  return result;
}
}  // namespace autoware::planning_validator
