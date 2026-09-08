#ifndef AUTOWARE_PLANNING_VALIDATOR_DIAGNOSTIC_RENDER_CORE_HPP_
#define AUTOWARE_PLANNING_VALIDATOR_DIAGNOSTIC_RENDER_CORE_HPP_

#include <autoware_planning_validator/PlanningValidatorStatus.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

#include <vector>

namespace autoware::planning_validator
{
struct DiagnosticRenderParameters
{
  int error_count_threshold{0};
  // This value originates in the trajectory plugin.  The current candidate
  // plugin ABI cannot transport it to the node, so node integration is not
  // authorized by this exploratory renderer.
  bool trajectory_override_all_errors{false};
};

// Candidate-only source table renderer.  It intentionally emits individual
// DiagnosticStatus entries (not an aggregate) for the 16 trajectory checks and
// three non-trajectory providers registered by the ROS2 source.
std::vector<diagnostic_msgs::DiagnosticStatus> render_source_shaped_diagnostics(
  const autoware_planning_validator::PlanningValidatorStatus & status,
  const DiagnosticRenderParameters & parameters);
}  // namespace autoware::planning_validator

#endif
