#ifndef AUTOWARE_PLANNING_VALIDATOR_REAR_COLLISION_CHECKER_PLUGIN_HPP_
#define AUTOWARE_PLANNING_VALIDATOR_REAR_COLLISION_CHECKER_PLUGIN_HPP_

#include <autoware/planning_validator/plugin_interface.hpp>
#include <autoware/planning_validator/rear_collision_decision_core.hpp>
#include <autoware/planning_validator/rear_motion_envelope_core.hpp>
#include <autoware/planning_validator/rear_object_assembly_core.hpp>
#include <autoware/planning_validator/rear_object_tracking_core.hpp>
#include <autoware/planning_validator/rear_pointcloud_core.hpp>
#include <autoware/planning_validator/rear_shift_activation_core.hpp>
#include <autoware/planning_validator/rear_turn_activation_core.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::planning_validator
{
// R003 establishes the installed ROS1 provider boundary. Missing source
// inputs, transform, route identity, or object identity remain fail-closed.
class RearCollisionCheckerRos1 final : public PluginInterface
{
public:
  void initialize(ros::NodeHandle & private_nh, const std::string & name) override;
  void validate(
    const TrajectoryCheckerInput & input,
    autoware_planning_validator::PlanningValidatorStatus & status) override;
  std::string module_name() const override;

private:
  std::string module_name_;
  // These defaults are the source packet's rear_collision_checker.param.yaml
  // values. initialize() allows the same values to be supplied through ROS1
  // private parameters without inventing an alternate safety configuration.
  RearPointCloudCoreParameters pointcloud_parameters_;
  RearMotionEnvelopeConstraints motion_envelope_constraints_;
  RearShiftActivationConfig shift_activation_config_;
  RearTurnActivationConfig turn_activation_config_;
  RearObjectTrackingConfig tracking_config_;
  std::unique_ptr<RearObjectTrackingCore> tracking_core_;
  RearRangeInput blind_spot_range_;
  RearObjectSafetyInput blind_spot_safety_;
  RearRangeInput adjacent_lane_range_;
  RearObjectSafetyInput adjacent_lane_safety_;
  double pointcloud_dead_zone_{0.3};
  double on_time_buffer_{0.5};
  double off_time_buffer_{1.5};
  RearSafetyDebounceState safety_debounce_;
};
}  // namespace autoware::planning_validator

#endif
