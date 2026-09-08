#include "autoware/planning_validator_trajectory_checker/trajectory_checker.hpp"

#include <pluginlib/class_list_macros.h>

#include <limits>
#include <vector>

namespace autoware::planning_validator
{
void TrajectoryCheckerRos1::initialize(ros::NodeHandle & private_nh, const std::string & name)
{
  module_name_ = name;
  TrajectoryCheckerCoreParameters parameters;
  bool interval_enabled = true, curvature_enabled = true, relative_angle_enabled = true;
  bool lateral_accel_enabled = true, min_lon_accel_enabled = true, max_lon_accel_enabled = true;
  bool lateral_jerk_enabled = true, steering_enabled = true, steering_rate_enabled = true;
  private_nh.param("trajectory_checker/interval/enable", interval_enabled, interval_enabled);
  private_nh.param("trajectory_checker/interval/threshold", parameters.interval_threshold, parameters.interval_threshold);
  private_nh.param("trajectory_checker/curvature/enable", curvature_enabled, curvature_enabled);
  private_nh.param("trajectory_checker/curvature/threshold", parameters.curvature_threshold, parameters.curvature_threshold);
  private_nh.param("trajectory_checker/relative_angle/enable", relative_angle_enabled, relative_angle_enabled);
  private_nh.param("trajectory_checker/relative_angle/threshold", parameters.relative_angle_threshold, parameters.relative_angle_threshold);
  private_nh.param("trajectory_checker/lateral_accel/enable", lateral_accel_enabled, lateral_accel_enabled);
  private_nh.param("trajectory_checker/lateral_accel/threshold", parameters.lateral_accel_threshold, parameters.lateral_accel_threshold);
  private_nh.param("trajectory_checker/min_lon_accel/enable", min_lon_accel_enabled, min_lon_accel_enabled);
  private_nh.param("trajectory_checker/min_lon_accel/threshold", parameters.min_lon_accel_threshold, parameters.min_lon_accel_threshold);
  private_nh.param("trajectory_checker/max_lon_accel/enable", max_lon_accel_enabled, max_lon_accel_enabled);
  private_nh.param("trajectory_checker/max_lon_accel/threshold", parameters.max_lon_accel_threshold, parameters.max_lon_accel_threshold);
  private_nh.param("trajectory_checker/lateral_jerk/enable", lateral_jerk_enabled, lateral_jerk_enabled);
  private_nh.param("trajectory_checker/lateral_jerk/threshold", parameters.lateral_jerk_threshold, parameters.lateral_jerk_threshold);
  private_nh.param("trajectory_checker/steering/enable", steering_enabled, steering_enabled);
  private_nh.param("trajectory_checker/steering/threshold", parameters.steering_threshold, parameters.steering_threshold);
  private_nh.param("trajectory_checker/steering_rate/enable", steering_rate_enabled, steering_rate_enabled);
  private_nh.param("trajectory_checker/steering_rate/threshold", parameters.steering_rate_threshold, parameters.steering_rate_threshold);
  private_nh.param(
    "trajectory_checker/trajectory_shift/override_error_diag",
    trajectory_shift_override_error_diag_, trajectory_shift_override_error_diag_);
  int default_handling_type = 0;
  private_nh.param("default_handling_type", default_handling_type, default_handling_type);
  const std::vector<std::string> handling_keys{
    "interval", "relative_angle", "curvature", "steering", "steering_rate", "lateral_jerk",
    "lateral_accel", "max_lon_accel", "min_lon_accel", "distance_deviation",
    "lon_distance_deviation", "velocity_deviation", "yaw_deviation", "trajectory_shift",
    "forward_trajectory_length"};
  handling_policy_valid_ = default_handling_type >= 0 && default_handling_type <= 2;
  for (const auto & key : handling_keys) {
    int value = default_handling_type;
    private_nh.param("trajectory_checker/" + key + "/handling_type", value, value);
    handling_types_.emplace(key, value);
    handling_policy_valid_ = handling_policy_valid_ && value >= 0 && value <= 2;
  }
  if (!handling_policy_valid_) ROS_ERROR("trajectory checker rejects invalid handling_type policy");
  private_nh.param("wheel_base_m", parameters.wheel_base_m, parameters.wheel_base_m);
  const auto disabled_threshold = std::numeric_limits<double>::infinity();
  if (!interval_enabled) parameters.interval_threshold = disabled_threshold;
  if (!curvature_enabled) parameters.curvature_threshold = disabled_threshold;
  if (!relative_angle_enabled) parameters.relative_angle_threshold = disabled_threshold;
  if (!lateral_accel_enabled) parameters.lateral_accel_threshold = disabled_threshold;
  if (!min_lon_accel_enabled) parameters.min_lon_accel_threshold = -disabled_threshold;
  if (!max_lon_accel_enabled) parameters.max_lon_accel_threshold = disabled_threshold;
  if (!lateral_jerk_enabled) parameters.lateral_jerk_threshold = disabled_threshold;
  if (!steering_enabled) parameters.steering_threshold = disabled_threshold;
  if (!steering_rate_enabled) parameters.steering_rate_threshold = disabled_threshold;
  checker_ = TrajectoryCheckerCore(parameters);
}

void TrajectoryCheckerRos1::validate(
  const TrajectoryCheckerInput & input, autoware_planning_validator::PlanningValidatorStatus & status)
{
  const auto checked = checker_.validate(input);
  // The current source parameter tree only grants this override to
  // trajectory_shift. Retain it as provider-local validation state and expose
  // it through the candidate diagnostic ABI.
  override_all_error_diag_ = trajectory_shift_override_error_diag_ && !checked.is_valid_trajectory_shift;
  requested_handling_ = -1;
  const auto request_if_invalid = [&](const std::string & key, const bool valid) {
    if (!valid) requested_handling_ = std::max(requested_handling_, handling_types_.at(key));
  };
  request_if_invalid("interval", checked.is_valid_interval);
  request_if_invalid("relative_angle", checked.is_valid_relative_angle);
  request_if_invalid("curvature", checked.is_valid_curvature);
  request_if_invalid("steering", checked.is_valid_steering);
  request_if_invalid("steering_rate", checked.is_valid_steering_rate);
  request_if_invalid("lateral_jerk", checked.is_valid_lateral_jerk);
  request_if_invalid("lateral_accel", checked.is_valid_lateral_acc);
  request_if_invalid("max_lon_accel", checked.is_valid_longitudinal_max_acc);
  request_if_invalid("min_lon_accel", checked.is_valid_longitudinal_min_acc);
  request_if_invalid("distance_deviation", checked.is_valid_distance_deviation);
  request_if_invalid("lon_distance_deviation", checked.is_valid_longitudinal_distance_deviation);
  request_if_invalid("velocity_deviation", checked.is_valid_velocity_deviation);
  request_if_invalid("yaw_deviation", checked.is_valid_yaw_deviation);
  request_if_invalid("trajectory_shift", checked.is_valid_trajectory_shift);
  request_if_invalid("forward_trajectory_length", checked.is_valid_forward_trajectory_length);
  status.stamp = checked.stamp;
  status.trajectory_size = checked.trajectory_size;
  status.is_valid_size = checked.is_valid_size;
  status.is_valid_finite_value = checked.is_valid_finite_value;
  status.is_valid_interval = checked.is_valid_interval;
  status.is_valid_relative_angle = checked.is_valid_relative_angle;
  status.is_valid_curvature = checked.is_valid_curvature;
  status.is_valid_lateral_acc = checked.is_valid_lateral_acc;
  status.is_valid_lateral_jerk = checked.is_valid_lateral_jerk;
  status.is_valid_longitudinal_max_acc = checked.is_valid_longitudinal_max_acc;
  status.is_valid_longitudinal_min_acc = checked.is_valid_longitudinal_min_acc;
  status.is_valid_steering = checked.is_valid_steering;
  status.is_valid_steering_rate = checked.is_valid_steering_rate;
  status.max_interval_distance = checked.max_interval_distance;
  status.max_relative_angle = checked.max_relative_angle;
  status.max_curvature = checked.max_curvature;
  status.max_lateral_acc = checked.max_lateral_acc;
  status.max_lateral_jerk = checked.max_lateral_jerk;
  status.max_longitudinal_acc = checked.max_longitudinal_acc;
  status.min_longitudinal_acc = checked.min_longitudinal_acc;
  status.max_steering = checked.max_steering;
  status.max_steering_rate = checked.max_steering_rate;
  status.is_valid_velocity_deviation = checked.is_valid_velocity_deviation;
  status.is_valid_distance_deviation = checked.is_valid_distance_deviation;
  status.is_valid_longitudinal_distance_deviation = checked.is_valid_longitudinal_distance_deviation;
  status.velocity_deviation = checked.velocity_deviation;
  status.distance_deviation = checked.distance_deviation;
  status.longitudinal_distance_deviation = checked.longitudinal_distance_deviation;
  status.is_valid_yaw_deviation = checked.is_valid_yaw_deviation;
  status.yaw_deviation = checked.yaw_deviation;
  status.is_valid_forward_trajectory_length = checked.is_valid_forward_trajectory_length;
  status.forward_trajectory_length_required = checked.forward_trajectory_length_required;
  status.forward_trajectory_length_measured = checked.forward_trajectory_length_measured;
  status.is_valid_trajectory_shift = checked.is_valid_trajectory_shift;
  status.lateral_shift = checked.lateral_shift;
  status.longitudinal_shift = checked.longitudinal_shift;
}

bool TrajectoryCheckerRos1::diagnostic_override_all_errors() const { return override_all_error_diag_; }

int TrajectoryCheckerRos1::requested_invalid_trajectory_handling() const
{
  return handling_policy_valid_ ? requested_handling_ : -2;
}

std::string TrajectoryCheckerRos1::module_name() const { return module_name_; }
}  // namespace autoware::planning_validator

PLUGINLIB_EXPORT_CLASS(autoware::planning_validator::TrajectoryCheckerRos1, autoware::planning_validator::PluginInterface)
