// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <autoware/behavior_path_planner/sub_planner_manager.hpp>
#include <boost/scope_exit.hpp>
namespace autoware::behavior_path_planner {
std::vector<SceneModulePtr> SubPlannerManager::getRequestModules(
  const BehaviorModuleOutput & previous_module_output,
  const std::vector<SceneModulePtr> & deleted_modules) const
{
  std::vector<SceneModulePtr> request_modules{};
  StopWatch<std::chrono::milliseconds> stop_watch;

  for (const auto & manager_ptr : manager_ptrs_) {
    stop_watch.tic(manager_ptr->name());
    BOOST_SCOPE_EXIT((&manager_ptr)(&processing_time_)(&stop_watch))
    {
      processing_time_.at(manager_ptr->name()) += stop_watch.toc(manager_ptr->name(), true);
    }
    BOOST_SCOPE_EXIT_END;

    if (const auto deleted_it = std::find_if(
          deleted_modules.begin(), deleted_modules.end(),
          [&](const auto & m) { return m->name() == manager_ptr->name(); });
        deleted_it != deleted_modules.end()) {
      continue;
    }

    // Condition 1:
    // the approved module queue is either
    // - consists of multiple simultaneous_executable_as_approved modules only
    // - consists of only 1 "not simultaneous_executable_as_approved" module
    const bool exclusive_module_exist_in_approved_pool = std::any_of(
      approved_module_ptrs_.begin(), approved_module_ptrs_.end(), [&](const SceneModulePtr & m) {
        return !getManager(m)->isSimultaneousExecutableAsApprovedModule();
      });
    const bool is_this_exclusive = !manager_ptr->isSimultaneousExecutableAsApprovedModule();
    const bool approved_pool_is_not_empty = !approved_module_ptrs_.empty();

    const bool is_this_not_joinable =
      exclusive_module_exist_in_approved_pool || (is_this_exclusive && approved_pool_is_not_empty);
    if (is_this_not_joinable) {
      continue;
    }

    /**
     * launch new candidate module.
     */
    {
      const auto name = manager_ptr->name();
      const auto find_same_name_module = [&name](const auto & m) { return m->name() == name; };
      const auto itr = std::find_if(
        candidate_module_ptrs_.begin(), candidate_module_ptrs_.end(), find_same_name_module);

      if (itr == candidate_module_ptrs_.end()) {
        if (manager_ptr->canLaunchNewModule()) {
          manager_ptr->updateIdleModuleInstance();
          if (manager_ptr->isExecutionRequested(previous_module_output)) {
            request_modules.emplace_back(std::dynamic_pointer_cast<SceneModuleInterface>(manager_ptr->takeIdleModule()));
          }
        }
        continue;
      }
    }

    /**
     * module already exist in candidate modules. check whether other modules can be launch as
     * candidate. if locked, break this loop.
     */
    {
      const auto name = manager_ptr->name();
      const auto find_block_module = [&name](const auto & m) {
        return m->name() == name && m->isLockedNewModuleLaunch();
      };
      const auto itr = std::find_if(
        candidate_module_ptrs_.begin(), candidate_module_ptrs_.end(), find_block_module);

      if (itr != candidate_module_ptrs_.end()) {
        request_modules.clear();
        request_modules.emplace_back(*itr);
        break;
      }
    }

    /**
     * module already exist. keep using it as candidate.
     */
    {
      const auto name = manager_ptr->name();
      const auto find_launched_module = [&name](const auto & m) { return m->name() == name; };
      const auto itr = std::find_if(
        candidate_module_ptrs_.begin(), candidate_module_ptrs_.end(), find_launched_module);

      if (itr != candidate_module_ptrs_.end()) {
        request_modules.emplace_back(*itr);
        continue;
      }
    }

    /**
     * same name module doesn't exist in candidate modules. launch new module.
     */
    {
      if (!manager_ptr->canLaunchNewModule()) {
        continue;
      }

      manager_ptr->updateIdleModuleInstance();
      if (!manager_ptr->isExecutionRequested(previous_module_output)) {
        continue;
      }

      request_modules.emplace_back(std::dynamic_pointer_cast<SceneModuleInterface>(manager_ptr->takeIdleModule()));
    }
  }

  return request_modules;
}

SceneModulePtr SubPlannerManager::selectHighestPriorityModule(
  std::vector<SceneModulePtr> & request_modules) const
{
  if (request_modules.empty()) {
    return {};
  }

  sortByPriority(request_modules);

  return request_modules.front();
}

void SubPlannerManager::clearApprovedModules()
{
  std::for_each(approved_module_ptrs_.begin(), approved_module_ptrs_.end(), [this](auto & m) {
    debug_info_.scene_status.emplace_back(
      m, SceneModuleUpdateInfo::Action::DELETE, "From Approved");
    deleteExpiredModules(m);
  });
  approved_module_ptrs_.clear();
}

void SubPlannerManager::updateCandidateModules(
  const std::vector<SceneModulePtr> & request_modules,
  const SceneModulePtr & highest_priority_module)
{
  const auto exist = [](const auto & module_ptr, const auto & module_ptrs) {
    const auto itr = std::find_if(
      module_ptrs.begin(), module_ptrs.end(),
      [&module_ptr](const auto & m) { return m->name() == module_ptr->name(); });

    return itr != module_ptrs.end();
  };

  /**
   * unregister expired modules
   */
  {
    const auto candidate_to_remove = [&](auto & itr) {
      if (!exist(itr, request_modules)) {
        deleteExpiredModules(itr);
        return true;
      }
      return itr->name() == highest_priority_module->name() &&
             !highest_priority_module->isWaitingApproval();
    };

    candidate_module_ptrs_.erase(
      std::remove_if(
        candidate_module_ptrs_.begin(), candidate_module_ptrs_.end(), candidate_to_remove),
      candidate_module_ptrs_.end());

    std::for_each(
      manager_ptrs_.begin(), manager_ptrs_.end(), [](const auto & m) { m->updateObserver(); });
  }

  /**
   * register running candidate modules
   */
  for (const auto & m : request_modules) {
    if (
      m->name() == highest_priority_module->name() &&
      !highest_priority_module->isWaitingApproval()) {
      continue;
    }

    if (!exist(m, candidate_module_ptrs_)) {
      candidate_module_ptrs_.push_back(m);
    }
  }

  /**
   * sort by priority. sorted_request_modules.front() is the highest priority module.
   */
  sortByPriority(candidate_module_ptrs_);
}

std::pair<SceneModulePtr, BehaviorModuleOutput> SubPlannerManager::runRequestModules(
  const std::vector<SceneModulePtr> & request_modules, const std::shared_ptr<PlannerData> & data,
  const BehaviorModuleOutput & previous_module_output)
{
  // modules that are filtered by simultaneous executable condition.
  std::vector<SceneModulePtr> executable_modules;

  // modules that are not approved after running yet.
  std::vector<SceneModulePtr> waiting_approved_modules;

  // modules that are already approved after running.
  std::vector<SceneModulePtr> already_approved_modules;

  // all request modules planning results.
  std::unordered_map<std::string, BehaviorModuleOutput> results;

  /**
   * sort by priority. sorted_request_modules.front() is the highest priority module.
   */
  auto sorted_request_modules = request_modules;
  sortByPriority(sorted_request_modules);

  // the candidate module queue is either
  // - consists of multiple simultaneous_executable_as_candidate modules only
  // - consists of only 1 "not simultaneous_executable_as_candidate" module
  for (const auto & module_ptr : sorted_request_modules) {
    // any module can join if executable_modules is empty
    const bool is_executable_modules_empty = executable_modules.empty();
    if (is_executable_modules_empty) {
      executable_modules.push_back(module_ptr);
      continue;
    }

    // if executable_module is not empty, only SimultaneousExecutableAsCandidate is joinable
    const bool is_this_cooperative =
      getManager(module_ptr)->isSimultaneousExecutableAsCandidateModule();
    const bool any_other_cooperative = std::any_of(
      executable_modules.begin(), executable_modules.end(), [&](const SceneModulePtr & m) {
        return getManager(m)->isSimultaneousExecutableAsCandidateModule();
      });

    if (is_this_cooperative && any_other_cooperative) {
      executable_modules.push_back(module_ptr);
    }
  }

  /**
   * run executable modules.
   */
  for (const auto & module_ptr : executable_modules) {
    const auto & manager_ptr = getManager(module_ptr);

    if (!manager_ptr->exist(module_ptr)) {
      manager_ptr->registerNewModule(
        module_ptr, previous_module_output);
    }

    results.emplace(module_ptr->name(), run(module_ptr, data, previous_module_output));
  }

  /**
   * remove expired modules.
   */
  {
    const auto remove_expired_modules = [this](auto & m) {
      if (m->getCurrentStatus() == ModuleStatus::FAILURE) {
        deleteExpiredModules(m);
        return true;
      }

      if (m->getCurrentStatus() == ModuleStatus::SUCCESS) {
        deleteExpiredModules(m);
        return true;
      }

      return false;
    };

    executable_modules.erase(
      std::remove_if(executable_modules.begin(), executable_modules.end(), remove_expired_modules),
      executable_modules.end());

    std::for_each(
      manager_ptrs_.begin(), manager_ptrs_.end(), [](const auto & m) { m->updateObserver(); });
  }

  /**
   * return null data if valid candidate module doesn't exist.
   */
  if (executable_modules.empty()) {
    clearCandidateModules();
    return std::make_pair(nullptr, BehaviorModuleOutput{});
  }

  /**
   * separate by approve condition.
   */
  std::for_each(executable_modules.begin(), executable_modules.end(), [&](const auto & m) {
    if (m->isWaitingApproval()) {
      waiting_approved_modules.push_back(m);
    } else {
      already_approved_modules.push_back(m);
    }
  });

  /**
   * choice highest priority module.
   */
  const auto module_ptr = [&]() -> SceneModulePtr {
    if (!already_approved_modules.empty()) {
      return selectHighestPriorityModule(already_approved_modules);
    }

    if (!waiting_approved_modules.empty()) {
      return selectHighestPriorityModule(waiting_approved_modules);
    }

    return nullptr;
  }();
  if (module_ptr == nullptr) {
    return std::make_pair(nullptr, BehaviorModuleOutput{});
  }

  /**
   * register candidate modules.
   */
  updateCandidateModules(executable_modules, module_ptr);

  return std::make_pair(module_ptr, results.at(module_ptr->name()));
}

BehaviorModuleOutput SubPlannerManager::run(
  const SceneModulePtr & module_ptr, const std::shared_ptr<PlannerData> & planner_data,
  const BehaviorModuleOutput & previous_module_output) const
{
  StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic(module_ptr->name());

  module_ptr->setData(planner_data);
  module_ptr->setPreviousModuleOutput(previous_module_output);

  module_ptr->lockRTCCommand();
  const auto result = [&]() {
    try { return module_ptr->run(); }
    catch (...) { module_ptr->unlockRTCCommand(); throw; }
  }();
  module_ptr->unlockRTCCommand();

  module_ptr->runPostProcess();

  module_ptr->updateCurrentState();

  module_ptr->publishObjectsOfInterestMarker();

  processing_time_.at(module_ptr->name()) += stop_watch.toc(module_ptr->name(), true);
  return result;
}

SlotOutput SubPlannerManager::runApprovedModules(
  const std::shared_ptr<PlannerData> & data, const BehaviorModuleOutput & upstream_slot_output,
  std::vector<SceneModulePtr> & deleted_modules)
{
  std::unordered_map<std::string, BehaviorModuleOutput> results;
  BehaviorModuleOutput output = upstream_slot_output;
  results.emplace("root", output);

  const bool is_candidate_plan_applied = true /* NOTE: not used in this process */;
  bool is_this_failed = false;
  bool is_this_waiting_approval = false;

  if (approved_module_ptrs_.empty()) {
    return SlotOutput{output, is_candidate_plan_applied, is_this_failed, is_this_waiting_approval};
  }

  // unlock only last approved module
  std::for_each(approved_module_ptrs_.begin(), approved_module_ptrs_.end(), [&](const auto & m) {
    m->lockOutputPath();
  });
  approved_module_ptrs_.back()->unlockOutputPath();

  /**
   * bootstrap approved module output
   */
  std::for_each(approved_module_ptrs_.begin(), approved_module_ptrs_.end(), [&](const auto & m) {
    output = run(m, data, output);
    results.emplace(m->name(), output);
  });

  const auto waiting_approval_modules_itr = std::find_if(
    approved_module_ptrs_.begin(), approved_module_ptrs_.end(),
    [](const auto & m) { return m->isWaitingApproval(); });

  if (waiting_approval_modules_itr != approved_module_ptrs_.end()) {
    is_this_waiting_approval = true;

    // only keep this module as candidate
    clearCandidateModules();
    candidate_module_ptrs_.push_back(*waiting_approval_modules_itr);

    // delete following result but keep the rest of the following modules
    std::for_each(
      waiting_approval_modules_itr, approved_module_ptrs_.end(),
      [&results](const auto & m) { results.erase(m->name()); });
    debug_info_.scene_status.emplace_back(
      *waiting_approval_modules_itr, SceneModuleUpdateInfo::Action::MOVE,
      "Back To Waiting Approval");
    approved_module_ptrs_.erase(waiting_approval_modules_itr);

    std::for_each(
      manager_ptrs_.begin(), manager_ptrs_.end(), [](const auto & m) { m->updateObserver(); });
  }

  /**
   * remove failure modules. these modules' outputs are discarded as invalid plan.
   */
  const auto failed_itr = std::find_if(
    approved_module_ptrs_.begin(), approved_module_ptrs_.end(),
    [](const auto & m) { return m->getCurrentStatus() == ModuleStatus::FAILURE; });
  if (failed_itr != approved_module_ptrs_.end()) {
    is_this_failed = true;

    // clear all candidates
    clearCandidateModules();

    // delete both subsequent result and modules
    std::for_each(failed_itr, approved_module_ptrs_.end(), [&](auto & m) {
      results.erase(m->name());
      debug_info_.scene_status.emplace_back(
        m, SceneModuleUpdateInfo::Action::DELETE, "From Approved");
      // NOTE(soblin): m is copied, so it is okay to call deleteExpiredModules
      deleted_modules.push_back(m);
      deleteExpiredModules(m);
    });
    approved_module_ptrs_.erase(failed_itr, approved_module_ptrs_.end());

    std::for_each(
      manager_ptrs_.begin(), manager_ptrs_.end(), [](const auto & m) { m->updateObserver(); });
  }

  if (approved_module_ptrs_.empty()) {
    return SlotOutput{
      results.at("root"), is_candidate_plan_applied, is_this_failed, is_this_waiting_approval};
  }

  // use the last module's output as approved modules planning result.
  const auto approved_modules_output = [&results, this]() {
    const auto itr = std::find_if(
      approved_module_ptrs_.rbegin(), approved_module_ptrs_.rend(),
      [&results](const auto & m) { return results.count(m->name()) != 0; });

    if (itr != approved_module_ptrs_.rend()) {
      return results.at((*itr)->name());
    }
    return results.at("root");
  }();

  // if lane change module has succeeded, update current route lanelet.
  if (std::any_of(approved_module_ptrs_.begin(), approved_module_ptrs_.end(), [](const auto & m) {
        return m->getCurrentStatus() == ModuleStatus::SUCCESS &&
               m->isCurrentRouteLaneletToBeReset();
      }))
    resetCurrentRouteLanelet(data);

  // remove success module immediately.
  for (auto success_itr = std::find_if(
         approved_module_ptrs_.begin(), approved_module_ptrs_.end(),
         [](const auto & m) { return m->getCurrentStatus() == ModuleStatus::SUCCESS; });
       success_itr != approved_module_ptrs_.end();
       /* success_itr++ */) {
    if ((*success_itr)->getCurrentStatus() == ModuleStatus::SUCCESS) {
      debug_info_.scene_status.emplace_back(
        *success_itr, SceneModuleUpdateInfo::Action::DELETE, "From Approved");
      deleted_modules.push_back(*success_itr);
      deleteExpiredModules(*success_itr);
      success_itr = approved_module_ptrs_.erase(success_itr);
    } else {
      success_itr++;
    }
  }

  std::for_each(
    manager_ptrs_.begin(), manager_ptrs_.end(), [](const auto & m) { m->updateObserver(); });

  return SlotOutput{
    approved_modules_output, is_candidate_plan_applied, is_this_failed, is_this_waiting_approval};
}

SlotOutput SubPlannerManager::propagateFull(
  const std::shared_ptr<PlannerData> & data, const SlotOutput & previous_slot_output)
{
  const size_t module_size = manager_ptrs_.size();
  const size_t max_iteration_num = static_cast<int>(module_size * (module_size + 1) / 2);

  bool is_waiting_approved_slot = previous_slot_output.is_upstream_waiting_approved;
  bool is_failed_approved_slot = false;
  auto output_path = previous_slot_output.valid_output;

  std::vector<SceneModulePtr> deleted_modules;
  for (size_t itr_num = 0; itr_num < max_iteration_num; ++itr_num) {
    const auto approved_module_result =
      runApprovedModules(data, previous_slot_output.valid_output, deleted_modules);
    const auto & approved_module_output = approved_module_result.valid_output;

    // these status needs to be propagated to downstream slots
    // if any of the slots returned following statuses, keep it
    is_waiting_approved_slot =
      is_waiting_approved_slot || approved_module_result.is_upstream_waiting_approved;
    is_failed_approved_slot =
      is_failed_approved_slot || approved_module_result.is_upstream_failed_approved;

    const auto request_modules = getRequestModules(approved_module_output, deleted_modules);
    if (request_modules.empty()) {
      // there is no module that needs to be launched
      return SlotOutput{
        approved_module_output, isAnyCandidateExclusive(), is_failed_approved_slot,
        is_waiting_approved_slot};
    }

    const auto [highest_priority_module, candidate_module_output] =
      runRequestModules(request_modules, data, approved_module_output);

    if (!highest_priority_module) {
      // there is no need to launch new module
      return SlotOutput{
        approved_module_output, isAnyCandidateExclusive(), is_failed_approved_slot,
        is_waiting_approved_slot};
    }

    if (highest_priority_module->isWaitingApproval()) {
      // there is no need to launch new module
      return SlotOutput{
        candidate_module_output, isAnyCandidateExclusive(), is_failed_approved_slot,
        is_waiting_approved_slot};
    }

    output_path = candidate_module_output;
    addApprovedModule(highest_priority_module);
    clearCandidateModules();
  }

  return SlotOutput{
    output_path, isAnyCandidateExclusive(), is_failed_approved_slot, is_waiting_approved_slot};
}

SlotOutput SubPlannerManager::propagateWithExclusiveCandidate(
  const std::shared_ptr<PlannerData> & data, const SlotOutput & previous_slot_output)
{
  std::vector<SceneModulePtr> deleted_modules;
  const auto approved_module_result =
    runApprovedModules(data, previous_slot_output.valid_output, deleted_modules);
  const auto & approved_module_output = approved_module_result.valid_output;

  // these status needs to be propagated to downstream slots
  // if any of the slots returned following statuses, keep it
  const bool is_waiting_approved_slot = previous_slot_output.is_upstream_waiting_approved |
                                        approved_module_result.is_upstream_waiting_approved;
  const bool is_failed_approved_slot = previous_slot_output.is_upstream_failed_approved |
                                       approved_module_result.is_upstream_failed_approved;

  // there is no module that needs to be launched
  return SlotOutput{
    approved_module_output, true, is_failed_approved_slot, is_waiting_approved_slot};
}

void SubPlannerManager::propagateWithFailedApproved()
{
  clearCandidateModules();
  clearApprovedModules();
}

SlotOutput SubPlannerManager::propagateWithWaitingApproved(
  const std::shared_ptr<PlannerData> & data, const SlotOutput & previous_slot_output)
{
  clearCandidateModules();

  return previous_slot_output.is_upstream_candidate_exclusive
           ? propagateWithExclusiveCandidate(data, previous_slot_output)
           : propagateFull(data, previous_slot_output);
}

}  // namespace autoware::behavior_path_planner
