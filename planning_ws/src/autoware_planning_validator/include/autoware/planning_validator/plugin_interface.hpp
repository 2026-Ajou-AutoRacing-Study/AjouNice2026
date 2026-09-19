#ifndef AUTOWARE_PLANNING_VALIDATOR_PLUGIN_INTERFACE_HPP_
#define AUTOWARE_PLANNING_VALIDATOR_PLUGIN_INTERFACE_HPP_

#include "autoware/planning_validator/trajectory_checker_core.hpp"

#include <ros/ros.h>

#include <string>

namespace autoware::planning_validator
{
class PluginInterface
{
public:
  virtual ~PluginInterface() = default;
  virtual void initialize(ros::NodeHandle & private_nh, const std::string & name) = 0;
  virtual void validate(
    const TrajectoryCheckerInput & input,
    autoware_planning_validator::PlanningValidatorStatus & status) = 0;
  virtual std::string module_name() const = 0;
  // Appended to keep every pre-existing virtual slot ABI-stable. Only the
  // rebuilt trajectory provider is queried for this source-specific state.
  virtual bool diagnostic_override_all_errors() const { return false; }
  // -1 means this provider does not request an override; 0..2 uses the
  // source handling order; -2 denotes an invalid provider configuration.
  virtual int requested_invalid_trajectory_handling() const { return -1; }
};
}  // namespace autoware::planning_validator

#endif
