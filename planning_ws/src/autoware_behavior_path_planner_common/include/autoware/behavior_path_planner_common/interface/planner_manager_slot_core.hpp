#pragma once

#include <autoware/behavior_path_planner_common/interface/scene_module_manager_core.hpp>
#include <autoware/behavior_path_planner_common/interface/scene_module_interface.hpp>

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{

// Fixed-order ROS1 execution subset of the upstream SubPlannerManager.  A slot
// runs already active modules first, then creates the first requesting manager
// in configured priority order.  Plugin discovery, simultaneous execution,
// RTC command locking and cross-slot propagation intentionally remain at the
// node-integration boundary.
class PlannerManagerSlotCore
{
public:
  explicit PlannerManagerSlotCore(std::vector<std::shared_ptr<SceneModuleManagerCore>> managers)
  : managers_(std::move(managers))
  {
  }

  BehaviorModuleOutput run(
    const std::shared_ptr<PlannerData> & data, const BehaviorModuleOutput & input)
  {
    BehaviorModuleOutput output = input;
    for (auto active = active_modules_.begin(); active != active_modules_.end();) {
      active->module->setData(data);
      active->module->setPreviousModuleOutput(output);
      output = runModule(active->module);
      if (isTerminal(active->module)) {
        active->module->onExit();
        active = active_modules_.erase(active);
      } else {
        ++active;
      }
    }

    if (!active_modules_.empty()) return output;

    for (const auto & manager : managers_) {
      if (!manager || !manager->canLaunchNewModule()) continue;
      manager->setData(data);
      manager->updateIdleModuleInstance();
      if (!manager->isExecutionRequested(output)) continue;
      auto module = manager->takeIdleModule();
      if (!module) continue;
      manager->registerNewModule(module, output);
      module->setData(data);
      module->setPreviousModuleOutput(output);
      output = runModule(module);
      if (isTerminal(module)) {
        module->onExit();
      } else {
        active_modules_.push_back({manager, std::move(module)});
      }
      return output;
    }
    return output;
  }

  void reset()
  {
    for (auto & active : active_modules_) active.module->onExit();
    active_modules_.clear();
    for (const auto & manager : managers_) {
      if (manager) manager->updateObserver();
    }
  }

  size_t activeModuleCount() const { return active_modules_.size(); }

private:
  // Preserve SubPlannerManager's provider lifecycle: RTC lock, run, unlock,
  // postProcess, state update, and object markers. Unlock on exceptions too.
  static BehaviorModuleOutput runModule(const std::shared_ptr<SceneModuleCore> & module)
  {
    const auto provider = std::dynamic_pointer_cast<SceneModuleInterface>(module);
    if (provider) provider->lockRTCCommand();
    BehaviorModuleOutput output;
    try {
      output = module->run();
    } catch (...) {
      if (provider) provider->unlockRTCCommand();
      throw;
    }
    if (provider) provider->unlockRTCCommand();
    module->runPostProcess();
    module->updateCurrentState();
    if (provider) provider->publishObjectsOfInterestMarker();
    return output;
  }

  struct ActiveModule
  {
    std::shared_ptr<SceneModuleManagerCore> manager;
    std::shared_ptr<SceneModuleCore> module;
  };

  static bool isTerminal(const std::shared_ptr<SceneModuleCore> & module)
  {
    return module->getCurrentStatus() == ModuleStatus::SUCCESS ||
           module->getCurrentStatus() == ModuleStatus::FAILURE;
  }

  std::vector<std::shared_ptr<SceneModuleManagerCore>> managers_;
  std::vector<ActiveModule> active_modules_;
};

}  // namespace autoware::behavior_path_planner
