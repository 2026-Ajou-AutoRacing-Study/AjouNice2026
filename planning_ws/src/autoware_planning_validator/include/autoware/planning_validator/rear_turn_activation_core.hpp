#ifndef AUTOWARE_PLANNING_VALIDATOR_REAR_TURN_ACTIVATION_CORE_HPP_
#define AUTOWARE_PLANNING_VALIDATOR_REAR_TURN_ACTIVATION_CORE_HPP_

#include "autoware/planning_validator/rear_shift_activation_core.hpp"

#include <autoware/route_handler/route_handler.hpp>

namespace autoware::planning_validator
{
struct RearTurnActivationInput
{
  const lanelet::ConstLanelets & current_lanes;
  const autoware_planning_msgs::Trajectory & trajectory;
  const nav_msgs::Odometry & current_kinematics;
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info;
  const RearMotionEnvelope & motion_envelope;
  const autoware::route_handler::RouteHandler * route_handler{nullptr};
  bool is_unsafe_holding{false};
};

struct RearTurnActivationConfig
{
  bool check_on_unstoppable{false};
  bool check_left{true};
  bool check_right{false};
  double yaw_threshold{0.78};
};

// Source-shaped check_turn_behavior render. It never creates a routing graph,
// sibling lane, or conflict point when the node-owned RouteHandler lacks one.
RearBehaviorActivationResult evaluate_rear_turn_activation(
  const RearTurnActivationInput & input, const RearTurnActivationConfig & config);
}  // namespace autoware::planning_validator

#endif
