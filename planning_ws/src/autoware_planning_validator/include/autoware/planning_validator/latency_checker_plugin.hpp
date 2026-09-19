#ifndef AUTOWARE_PLANNING_VALIDATOR_LATENCY_CHECKER_PLUGIN_HPP_
#define AUTOWARE_PLANNING_VALIDATOR_LATENCY_CHECKER_PLUGIN_HPP_

#include "autoware/planning_validator/plugin_interface.hpp"

namespace autoware::planning_validator
{
class LatencyCheckerPlugin final : public PluginInterface
{
public:
  void initialize(ros::NodeHandle & private_nh, const std::string & name) override;
  void validate(
    const TrajectoryCheckerInput & input,
    autoware_planning_validator::PlanningValidatorStatus & status) override;
  std::string module_name() const override;

private:
  std::string module_name_;
  bool enabled_{true};
  double threshold_sec_{1.0};
};
}  // namespace autoware::planning_validator

#endif
