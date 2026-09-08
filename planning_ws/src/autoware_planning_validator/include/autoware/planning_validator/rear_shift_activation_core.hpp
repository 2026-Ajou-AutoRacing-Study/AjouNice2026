#ifndef AUTOWARE_PLANNING_VALIDATOR_REAR_SHIFT_ACTIVATION_CORE_HPP_
#define AUTOWARE_PLANNING_VALIDATOR_REAR_SHIFT_ACTIVATION_CORE_HPP_

#include "autoware/planning_validator/rear_motion_envelope_core.hpp"

#include <autoware_planning_msgs/Trajectory.h>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <lanelet2_core/primitives/Lanelet.h>
#include <nav_msgs/Odometry.h>

namespace autoware::planning_validator
{
enum class RearBehavior { NONE = 0, SHIFT_LEFT, SHIFT_RIGHT, TURN_LEFT, TURN_RIGHT };

struct RearShiftActivationInput
{
  const lanelet::ConstLanelets & current_lanes;
  const autoware_planning_msgs::Trajectory & trajectory;
  const nav_msgs::Odometry & current_kinematics;
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info;
  const RearMotionEnvelope & motion_envelope;
  bool is_unsafe_holding{false};
};

struct RearShiftActivationConfig
{
  bool check_on_unstoppable{false};
};

struct RearBehaviorActivationResult
{
  RearBehavior behavior{RearBehavior::NONE};
  double distance_to_conflict{0.0};
};

RearBehaviorActivationResult evaluate_rear_shift_activation(
  const RearShiftActivationInput & input, const RearShiftActivationConfig & config);
}  // namespace autoware::planning_validator

#endif
