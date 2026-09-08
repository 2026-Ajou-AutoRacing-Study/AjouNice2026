#pragma once

#include <string>

namespace autoware::behavior_path_planner
{

// This is the dependency-free state-transition kernel of the upstream
// SceneModuleInterface.  RTC publishing, marker publishing and planner output
// ownership deliberately remain in their respective integration slices.
enum class ModuleStatus {
  IDLE = 0,
  RUNNING = 1,
  WAITING_APPROVAL = 2,
  SUCCESS = 3,
  FAILURE = 4,
};

struct ApprovalState {
  bool request_registered{false};
  bool request_approved{false};
  bool request_terminated{false};
  // A provider with more than one RTC endpoint needs the exact aggregate
  // predicates from the upstream SceneModuleInterface, rather than a lossy
  // reconstruction from one arbitrary endpoint.  Keep the first three fields
  // for dependency-free lifecycle tests and use these only when a provider
  // supplies the complete aggregate state.
  bool has_provider_aggregate{false};
  bool has_unapproved_request{false};
  bool can_leave_waiting_approval{false};
};

class SceneModuleLifecycle
{
public:
  explicit SceneModuleLifecycle(
    std::string name, const ModuleStatus initial_state = ModuleStatus::IDLE)
  : name_(std::move(name)), current_state_(initial_state)
  {
  }

  ModuleStatus update(
    const bool can_transit_success, const bool can_transit_failure,
    const ModuleStatus initial_state = ModuleStatus::RUNNING)
  {
    if (current_state_ == ModuleStatus::IDLE) {
      current_state_ = initial_state;
      return current_state_;
    }
    if (current_state_ == ModuleStatus::RUNNING) {
      if (can_transit_success) return current_state_ = ModuleStatus::SUCCESS;
      if (can_transit_failure) return current_state_ = ModuleStatus::FAILURE;
      if (approval_.has_provider_aggregate && approval_.has_unapproved_request) {
        return current_state_ = ModuleStatus::WAITING_APPROVAL;
      }
      if (approval_.has_provider_aggregate) return current_state_;
      if (approval_.request_registered && !approval_.request_approved && !approval_.request_terminated) {
        return current_state_ = ModuleStatus::WAITING_APPROVAL;
      }
      return current_state_;
    }
    if (current_state_ == ModuleStatus::WAITING_APPROVAL) {
      if (can_transit_success) return current_state_ = ModuleStatus::SUCCESS;
      if (can_transit_failure) return current_state_ = ModuleStatus::FAILURE;
      if (approval_.has_provider_aggregate && approval_.can_leave_waiting_approval) {
        return current_state_ = ModuleStatus::RUNNING;
      }
      if (approval_.has_provider_aggregate) return current_state_;
      if (!approval_.request_registered || approval_.request_approved) {
        return current_state_ = ModuleStatus::RUNNING;
      }
      return current_state_;
    }
    return current_state_;
  }

  void setApprovalState(const ApprovalState & approval) { approval_ = approval; }
  void waitApproval() { explicit_waiting_approval_ = true; }
  void clearWaitingApproval() { explicit_waiting_approval_ = false; }
  bool isWaitingApproval() const
  {
    return explicit_waiting_approval_ || current_state_ == ModuleStatus::WAITING_APPROVAL;
  }

  void lockNewModuleLaunch() { new_module_launch_locked_ = true; }
  void unlockNewModuleLaunch() { new_module_launch_locked_ = false; }
  bool isNewModuleLaunchLocked() const { return new_module_launch_locked_; }

  // Mirrors SceneModuleInterface::onExit state cleanup.  RTC status publication
  // is intentionally not represented here because it requires the RTC provider.
  void onExit()
  {
    clearWaitingApproval();
    new_module_launch_locked_ = false;
    output_path_locked_ = false;
  }

  void lockOutputPath() { output_path_locked_ = true; }
  void unlockOutputPath() { output_path_locked_ = false; }
  bool isOutputPathLocked() const { return output_path_locked_; }
  ModuleStatus status() const { return current_state_; }
  const std::string & name() const { return name_; }

private:
  std::string name_;
  ModuleStatus current_state_{ModuleStatus::IDLE};
  ApprovalState approval_{};
  bool explicit_waiting_approval_{false};
  bool new_module_launch_locked_{false};
  bool output_path_locked_{false};
};

}  // namespace autoware::behavior_path_planner
