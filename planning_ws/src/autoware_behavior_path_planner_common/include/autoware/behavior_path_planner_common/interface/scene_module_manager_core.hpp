#pragma once

#include <autoware/behavior_path_planner_common/interface/scene_module_core.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{

// ROS1 ownership subset of upstream SceneModuleManagerInterface.  It preserves
// idle-module creation, data/output injection, observer lifetime and exit
// ordering, while deliberately leaving RTC/marker/planning-factor publication
// to the provider-bound interface closure.
class SceneModuleManagerCore
{
public:
  explicit SceneModuleManagerCore(std::string name) : name_(std::move(name)) {}
  virtual ~SceneModuleManagerCore() = default;

  void setData(const std::shared_ptr<PlannerData> & data) { planner_data_ = data; }

  void updateIdleModuleInstance()
  {
    if (idle_module_) {
      // Preserve SceneModuleManagerInterface: an already-owned idle module
      // receives its entry hook on subsequent update cycles.
      idle_module_->onEntry();
      return;
    }
    idle_module_ = createNewSceneModuleInstance();
  }

  std::shared_ptr<SceneModuleCore> takeIdleModule()
  {
    if (!idle_module_) return {};
    return std::shared_ptr<SceneModuleCore>(std::move(idle_module_));
  }

  bool isExecutionRequested(const BehaviorModuleOutput & previous_output)
  {
    if (!idle_module_) return false;
    idle_module_->setData(planner_data_);
    idle_module_->setPreviousModuleOutput(previous_output);
    return idle_module_->requestsExecution();
  }

  void registerNewModule(
    const std::shared_ptr<SceneModuleCore> & module,
    const BehaviorModuleOutput & previous_output)
  {
    if (!module) return;
    module->setData(planner_data_);
    module->setPreviousModuleOutput(previous_output);
    module->onEntry();
    observers_.push_back(module);
  }

  void updateObserver()
  {
    observers_.erase(
      std::remove_if(observers_.begin(), observers_.end(),
        [](const std::weak_ptr<SceneModuleCore> & observer) { return observer.expired(); }),
      observers_.end());
  }

  bool canLaunchNewModule() const { return observers_.empty(); }
  bool exist(const std::shared_ptr<SceneModuleCore> & module) const
  {
    return std::any_of(observers_.begin(), observers_.end(), [&](const auto & observer) {
      return !observer.expired() && observer.lock() == module;
    });
  }

  void reset()
  {
    for (const auto & observer : observers_) {
      if (const auto module = observer.lock()) module->onExit();
    }
    observers_.clear();
    if (idle_module_) {
      idle_module_->onExit();
      idle_module_.reset();
    }
  }

  const std::string & name() const { return name_; }
  size_t observerCount() const { return observers_.size(); }
  std::vector<std::weak_ptr<SceneModuleCore>> observerSnapshot() const { return observers_; }

protected:
  const std::shared_ptr<const PlannerData> & plannerData() const { return planner_data_; }
  SceneModuleCore * idleModule() const { return idle_module_.get(); }

  std::vector<std::shared_ptr<SceneModuleCore>> activeModules() const
  {
    std::vector<std::shared_ptr<SceneModuleCore>> modules;
    modules.reserve(observers_.size());
    for (const auto & observer : observers_) {
      if (const auto module = observer.lock()) modules.push_back(module);
    }
    return modules;
  }

  virtual std::unique_ptr<SceneModuleCore> createNewSceneModuleInstance() = 0;

private:
  std::string name_;
  std::shared_ptr<const PlannerData> planner_data_;
  std::vector<std::weak_ptr<SceneModuleCore>> observers_;
  std::unique_ptr<SceneModuleCore> idle_module_;
};

}  // namespace autoware::behavior_path_planner
