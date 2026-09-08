#ifndef AUTOWARE_PLANNING_VALIDATOR_LATENCY_CHECKER_LATENCY_CHECKER_HPP_
#define AUTOWARE_PLANNING_VALIDATOR_LATENCY_CHECKER_LATENCY_CHECKER_HPP_

#include <autoware/planning_validator/plugin_interface.hpp>

#include <string>

namespace autoware::planning_validator
{
// The suffix is intentional.  R002's base candidate still exposes an
// in-package provisional LatencyChecker implementation.  A distinct C++
// symbol lets pluginlib load this external provider without duplicate-class
// ambiguity; manager configuration will switch only after this provider is
// independently accepted.
class LatencyCheckerRos1 final : public PluginInterface
{
public:
  void initialize(ros::NodeHandle & private_nh, const std::string & name) override;
  void validate(
    const TrajectoryCheckerInput & input,
    autoware_planning_validator::PlanningValidatorStatus & status) override;
  std::string module_name() const override;

private:
  std::string module_name_;
  bool enable_latency_check_{true};
  double latency_threshold_{1.0};
};
}  // namespace autoware::planning_validator

#endif
