#ifndef AUTOWARE_PLANNING_VALIDATOR_TRAJECTORY_CHECKER_PLUGIN_HPP_
#define AUTOWARE_PLANNING_VALIDATOR_TRAJECTORY_CHECKER_PLUGIN_HPP_

#include "autoware/planning_validator/plugin_interface.hpp"
#include "autoware/planning_validator/trajectory_checker_core.hpp"

namespace autoware::planning_validator
{
class TrajectoryCheckerPlugin final : public PluginInterface
{
public:
  void initialize(ros::NodeHandle & private_nh, const std::string & name) override;
  void validate(
    const TrajectoryCheckerInput & input,
    autoware_planning_validator::PlanningValidatorStatus & status) override;
  std::string module_name() const override;

private:
  std::string module_name_;
  TrajectoryCheckerCore checker_;
};
}  // namespace autoware::planning_validator

#endif
