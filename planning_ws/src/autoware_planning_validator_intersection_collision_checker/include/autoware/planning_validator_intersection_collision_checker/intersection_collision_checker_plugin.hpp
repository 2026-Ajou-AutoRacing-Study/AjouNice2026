#pragma once

#include <autoware/planning_validator/plugin_interface.hpp>
#include <autoware/planning_validator_intersection_collision_checker/parameters_ros1.hpp>
#include <autoware/planning_validator_intersection_collision_checker/active_turn_collision_core_ros1.hpp>

#include <string>
#include <vector>

namespace autoware::planning_validator
{
// Candidate ABI adapter.  It is intentionally fail-closed until the complete
// source right/left target-lane discovery and point-in-lane pipeline below it
// are connected; it must not report an unimplemented safety check as valid.
class IntersectionCollisionCheckerRos1 final : public PluginInterface
{
public:
  void initialize(ros::NodeHandle & private_nh, const std::string & name) override;
  void validate(const TrajectoryCheckerInput & input,
    autoware_planning_validator::PlanningValidatorStatus & status) override;
  std::string module_name() const override { return module_name_; }

private:
  std::string module_name_;
  intersection_collision_checker_ros1::Params params_{};
  bool parameters_ready_{false};
  intersection_collision_checker::ActiveTurnCollisionCore collision_core_;
  std::vector<lanelet::Id> previous_collision_lanes_;
  ros::Time last_valid_time_;
  ros::Time last_invalid_time_;
};
}  // namespace autoware::planning_validator
