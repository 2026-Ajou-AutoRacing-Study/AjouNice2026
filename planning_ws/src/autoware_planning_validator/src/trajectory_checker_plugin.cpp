#include "autoware/planning_validator/trajectory_checker_plugin.hpp"

#include <pluginlib/class_list_macros.h>

namespace autoware::planning_validator
{
void TrajectoryCheckerPlugin::initialize(ros::NodeHandle & private_nh, const std::string & name)
{
  module_name_ = name;
  TrajectoryCheckerCoreParameters parameters;
  private_nh.param("wheel_base_m", parameters.wheel_base_m, parameters.wheel_base_m);
  checker_ = TrajectoryCheckerCore(parameters);
}

void TrajectoryCheckerPlugin::validate(
  const TrajectoryCheckerInput & input,
  autoware_planning_validator::PlanningValidatorStatus & status)
{
  const auto checked = checker_.validate(input);
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
}

std::string TrajectoryCheckerPlugin::module_name() const { return module_name_; }
}  // namespace autoware::planning_validator

PLUGINLIB_EXPORT_CLASS(
  autoware::planning_validator::TrajectoryCheckerPlugin,
  autoware::planning_validator::PluginInterface)
