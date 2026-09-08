#include "autoware/planning_validator/latency_checker_plugin.hpp"

#include <pluginlib/class_list_macros.h>

namespace autoware::planning_validator
{
void LatencyCheckerPlugin::initialize(ros::NodeHandle & private_nh, const std::string & name)
{
  module_name_ = name;
  private_nh.param("latency_checker/enable", enabled_, true);
  private_nh.param("latency_checker/threshold", threshold_sec_, 1.0);
}

void LatencyCheckerPlugin::validate(
  const TrajectoryCheckerInput & input,
  autoware_planning_validator::PlanningValidatorStatus & status)
{
  if (!enabled_) {
    status.is_valid_latency = true;
    return;
  }
  status.latency = (ros::Time::now() - input.trajectory.header.stamp).toSec();
  status.is_valid_latency = status.latency <= threshold_sec_;
}

std::string LatencyCheckerPlugin::module_name() const { return module_name_; }
}  // namespace autoware::planning_validator

PLUGINLIB_EXPORT_CLASS(
  autoware::planning_validator::LatencyCheckerPlugin,
  autoware::planning_validator::PluginInterface)
