#pragma once

#include <autoware/behavior_path_planner_common/data_types.hpp>
#include <autoware/behavior_path_planner_common/interface/scene_module_lifecycle.hpp>
#include <autoware/behavior_path_planner_common/planner_data.hpp>

#include <cmath>
#include <any>
#include <memory>
#include <stdexcept>
#include <string>

namespace autoware::behavior_path_planner
{
class SceneModuleVisitor;

// Executable subset of upstream SceneModuleInterface needed by PlannerManager:
// data ingress, state transition, normal/approval planning split and non-empty
// output invariant. RTC, marker and planning-factor side effects remain a later
// provider-bound interface slice.
class SceneModuleCore
{
public:
  explicit SceneModuleCore(std::string name, const ModuleStatus initial = ModuleStatus::IDLE)
  : name_(std::move(name)), lifecycle_(name_, initial)
  {
  }
  virtual ~SceneModuleCore() = default;

  // Providers such as LaneChange must receive the same planner data in their
  // module-local state before requestsExecution() dereferences route data.
  // A non-virtual base function silently bypassed that provider handoff when
  // the slot stored the object through SceneModuleCore.
  virtual void setData(const std::shared_ptr<const PlannerData> & data) { planner_data_ = data; }
  void setPreviousModuleOutput(const BehaviorModuleOutput & output) { previous_output_ = output; }
  const BehaviorModuleOutput & getPreviousModuleOutput() const { return previous_output_; }
  const autoware_internal_planning_msgs::PathWithLaneId & getPathCandidate() const
  {
    return path_candidate_;
  }
  const autoware_internal_planning_msgs::PathWithLaneId & getPathReference() const
  {
    return path_reference_;
  }
  void resetPathCandidate() { path_candidate_ = autoware_internal_planning_msgs::PathWithLaneId{}; }
  void resetPathReference() { path_reference_ = autoware_internal_planning_msgs::PathWithLaneId{}; }
  const std::shared_ptr<const PlannerData> & plannerData() const { return planner_data_; }
  const std::string & name() const { return name_; }
  ModuleStatus getCurrentStatus() const { return lifecycle_.status(); }
  bool isWaitingApproval() const { return lifecycle_.isWaitingApproval(); }

  // Manager-facing entrypoints.  Provider side effects (RTC, marker and
  // planning-factor publication) remain outside this ROS1-independent core.
  void onEntry() { processOnEntry(); }
  void runPostProcess() { postProcess(); }
  bool executionReady() const { return isExecutionReady(); }
  void visit(const std::shared_ptr<SceneModuleVisitor> & visitor) const { acceptVisitor(visitor); }
  bool requestsExecution()
  {
    updateData();
    return isExecutionRequested();
  }

  void updateCurrentState()
  {
    refreshApprovalState();
    lifecycle_.update(canTransitSuccessState(), canTransitFailureState(), setInitState());
  }

  virtual BehaviorModuleOutput run()
  {
    updateData();
    BehaviorModuleOutput output = isWaitingApproval() ? planWaitingApproval() : plan();
    if (output.path.points.empty()) {
      throw std::invalid_argument("[" + name_ + "] scene module returned an empty path");
    }
    return output;
  }

  void waitApproval() { lifecycle_.waitApproval(); }
  void clearWaitingApproval() { lifecycle_.clearWaitingApproval(); }
  void lockNewModuleLaunch() { lifecycle_.lockNewModuleLaunch(); }
  void unlockNewModuleLaunch() { lifecycle_.unlockNewModuleLaunch(); }
  bool isLockedNewModuleLaunch() const { return lifecycle_.isNewModuleLaunchLocked(); }
  void lockOutputPath() { lifecycle_.lockOutputPath(); }
  void unlockOutputPath() { lifecycle_.unlockOutputPath(); }
  virtual bool isCurrentRouteLaneletToBeReset() const { return false; }
  bool isOutputPathLocked() const { return lifecycle_.isOutputPathLocked(); }
  virtual void onExit() { lifecycle_.onExit(); processOnExit(); }

protected:
  // A provider-bearing module refreshes its RTC-derived approval predicates
  // before every lifecycle transition.  The core remains usable without ROS
  // providers, where the default no-op preserves the dependency-free tests.
  virtual void refreshApprovalState() {}
  void setApprovalState(const ApprovalState & approval) { lifecycle_.setApprovalState(approval); }

  geometry_msgs::Point getEgoPosition() const { return planner_data_->self_odometry->pose.pose.position; }
  geometry_msgs::Pose getEgoPose() const { return planner_data_->self_odometry->pose.pose; }
  geometry_msgs::Twist getEgoTwist() const { return planner_data_->self_odometry->twist.twist; }
  double getEgoSpeed() const { return std::abs(getEgoTwist().linear.x); }

  virtual bool isExecutionRequested() const = 0;
  virtual bool isExecutionReady() const = 0;
  virtual bool canTransitSuccessState() = 0;
  virtual bool canTransitFailureState() = 0;
  virtual void updateData() {}
  virtual void processOnEntry() {}
  virtual void postProcess() {}
  virtual void processOnExit() {}
  virtual void updateModuleParams(const std::any &) {}
  virtual void acceptVisitor(const std::shared_ptr<SceneModuleVisitor> &) const {}
  virtual ModuleStatus setInitState() const { return ModuleStatus::RUNNING; }
  virtual CandidateOutput planCandidate() const = 0;
  virtual BehaviorModuleOutput plan() = 0;

  virtual BehaviorModuleOutput planWaitingApproval()
  {
    path_candidate_ = planCandidate().path_candidate;
    path_reference_ = previous_output_.reference_path;
    return previous_output_;
  }

  autoware_internal_planning_msgs::PathWithLaneId path_candidate_;
  autoware_internal_planning_msgs::PathWithLaneId path_reference_;

private:
  std::string name_;
  SceneModuleLifecycle lifecycle_;

protected:
  std::shared_ptr<const PlannerData> planner_data_;

private:
  BehaviorModuleOutput previous_output_;
};

}  // namespace autoware::behavior_path_planner
