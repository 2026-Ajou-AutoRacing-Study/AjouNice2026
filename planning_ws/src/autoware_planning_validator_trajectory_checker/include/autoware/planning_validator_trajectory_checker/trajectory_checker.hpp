#ifndef AUTOWARE_PLANNING_VALIDATOR_TRAJECTORY_CHECKER_TRAJECTORY_CHECKER_HPP_
#define AUTOWARE_PLANNING_VALIDATOR_TRAJECTORY_CHECKER_TRAJECTORY_CHECKER_HPP_

#include <autoware/planning_validator/plugin_interface.hpp>
#include <autoware/planning_validator/trajectory_checker_core.hpp>

#include <string>
#include <map>

namespace autoware::planning_validator
{
// R002 already renders the source-shaped static checker in
// TrajectoryCheckerCore.  This provider restores the upstream package/plugin
// boundary without claiming unrendered diagnostics or manager context.
class TrajectoryCheckerRos1 final : public PluginInterface
{
public:
  void initialize(ros::NodeHandle & private_nh, const std::string & name) override;
  void validate(const TrajectoryCheckerInput & input, autoware_planning_validator::PlanningValidatorStatus & status) override;
  bool diagnostic_override_all_errors() const override;
  int requested_invalid_trajectory_handling() const override;
  std::string module_name() const override;

private:
  std::string module_name_;
  TrajectoryCheckerCore checker_;
  bool trajectory_shift_override_error_diag_{true};
  bool override_all_error_diag_{false};
  bool handling_policy_valid_{true};
  int requested_handling_{-1};
  std::map<std::string, int> handling_types_;
};
}  // namespace autoware::planning_validator

#endif
