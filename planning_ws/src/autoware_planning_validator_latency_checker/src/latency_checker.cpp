#include "autoware/planning_validator_latency_checker/latency_checker.hpp"

#include <pluginlib/class_list_macros.h>

namespace autoware::planning_validator
{
void LatencyCheckerRos1::initialize(ros::NodeHandle & private_nh, const std::string & name)
{
  module_name_ = name;
  private_nh.param("latency_checker/enable", enable_latency_check_, true);
  private_nh.param("latency_checker/threshold", latency_threshold_, 1.0);
}

void LatencyCheckerRos1::validate(
  const TrajectoryCheckerInput & input,
  autoware_planning_validator::PlanningValidatorStatus & status)
{
  // This matches the upstream guard: when disabled it leaves the aggregate
  // status untouched.  The manager's fail-closed defaults remain responsible
  // for unrendered checker results.
  if (!enable_latency_check_) return;

  status.latency = (ros::Time::now() - input.trajectory.header.stamp).toSec();
  status.is_valid_latency = status.latency <= latency_threshold_;
}

std::string LatencyCheckerRos1::module_name() const { return module_name_; }
}  // namespace autoware::planning_validator

PLUGINLIB_EXPORT_CLASS(
  autoware::planning_validator::LatencyCheckerRos1,
  autoware::planning_validator::PluginInterface)
