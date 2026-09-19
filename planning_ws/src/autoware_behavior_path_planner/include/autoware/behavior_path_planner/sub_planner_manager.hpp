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

#pragma once
#include <autoware/behavior_path_planner_common/interface/scene_module_interface.hpp>
#include <autoware/behavior_path_planner_common/interface/scene_module_manager_interface.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <optional>
#include <stdexcept>
namespace autoware::behavior_path_planner {
using autoware_utils::StopWatch;
using SceneModulePtr = std::shared_ptr<SceneModuleInterface>;
using SceneModuleManagerPtr = std::shared_ptr<SceneModuleManagerInterface>;
struct SceneModuleUpdateInfo
{
  enum Action {
    ADD = 0,
    DELETE,
    MOVE,
  };
  explicit SceneModuleUpdateInfo(
    const SceneModulePtr & module_ptr, const Action & action, const std::string & description)
  : status(module_ptr->getCurrentStatus()),
    action(action),
    module_name(module_ptr->name()),
    description(description)
  {
  }

  explicit SceneModuleUpdateInfo(
    const std::string & name, const Action & action, const ModuleStatus & status,
    const std::string & description)
  : status(status), action(action), module_name(name), description(description)
  {
  }

  ModuleStatus status;

  Action action;

  std::string module_name;

  std::string description;
};

enum SlotStatus {
  NORMAL = 0,
  UPSTREAM_APPROVED_FAILED,
  UPSTREAM_WAITING_APPROVED,
  UPSTREAM_EXCLUSIVE_CANDIDATE
};

struct ModuleUpdateInfo
{
  std::vector<SceneModuleUpdateInfo> scene_status;
  std::vector<SlotStatus> slot_status;
};

struct SceneModuleStatus
{
  explicit SceneModuleStatus(const std::string & n) : module_name(n) {}

  std::string module_name;

  bool is_execution_ready{false};
  bool is_waiting_approval{false};

  ModuleStatus status{ModuleStatus::SUCCESS};
};

struct SlotOutput
{
  BehaviorModuleOutput valid_output;

  // if candidate module is running, valid_output contains the planning by candidate module. In
  // that case, downstream slots will just run approved modules and do not try to launch new
  // modules
  bool is_upstream_candidate_exclusive{false};

  // if this slot failed, downstream slots need to refresh approved/candidate modules and just
  // forward valid_output of this slot output
  bool is_upstream_failed_approved{false};

  // if the approved module in this slot returned to isWaitingApproval, downstream slots need to
  // refresh candidate once
  bool is_upstream_waiting_approved{false};
};

class SubPlannerManager
{
public:
  explicit SubPlannerManager(
    std::shared_ptr<std::optional<lanelet::ConstLanelet>> lanelet,
    std::unordered_map<std::string, double> & processing_time, ModuleUpdateInfo & debug_info)
  : current_route_lanelet_(lanelet),
    processing_time_(std::ref(processing_time)),
    debug_info_(std::ref(debug_info))
  {
  }

  void addSceneModuleManager(const SceneModuleManagerPtr module_ptr)
  {
    manager_ptrs_.push_back(module_ptr);
    // NOTE(Mamoru Sobue): SceneModuleManagerPtr->name() == SceneModulePtr->name()
    module_priorities_[module_ptr->name()] = module_priorities_.size();
  }

  const std::vector<SceneModuleManagerPtr> & getSceneModuleManager() const { return manager_ptrs_; }

  /**
   * @pre previous_slot_output.is_upstream_candidate_exclusive is false
   * @pre previous_slot_output.is_upstream_failed_approved is false
   */
  SlotOutput propagateFull(
    const std::shared_ptr<PlannerData> & planner_data, const SlotOutput & previous_slot_output);

  /**
   * @brief run approved modules without launching new modules(candidates)
   * @pre previous_slot_output.is_upstream_candidate_exclusive is true
   * @post is_upstream_candidate_exclusive of output is true
   * @post candidate_module does not increase
   */
  SlotOutput propagateWithExclusiveCandidate(
    const std::shared_ptr<PlannerData> & planner_data, const SlotOutput & previous_slot_output);

  /**
   * @brief delete approved/candidate modules and just forward upstream output
   * @post candidate_module and approved_module get empty
   */
  void propagateWithFailedApproved();

  /**
   * @pre previous_slot_output.is_upstream_waiting_approved is true
   * @post is_upstream_waiting_approved of output is true
   */
  SlotOutput propagateWithWaitingApproved(
    const std::shared_ptr<PlannerData> & planner_data, const SlotOutput & previous_slot_output);

  template <typename F>
  bool isAnyApprovedPred(F && pred) const
  {
    return std::any_of(
      approved_module_ptrs_.begin(), approved_module_ptrs_.end(),
      [&](const auto & m) { return pred(m); });
  }

  template <typename F>
  bool isAnyCandidatePred(F && pred) const
  {
    return std::any_of(
      candidate_module_ptrs_.begin(), candidate_module_ptrs_.end(),
      [&](const auto & m) { return pred(m); });
  }

  const std::vector<SceneModulePtr> & approved_modules() const { return approved_module_ptrs_; }

  const std::vector<SceneModulePtr> & candidate_modules() const { return candidate_module_ptrs_; }

  /**
   * @brief reset the current route lanelet to be the closest lanelet within the route
   * @param planner data.
   */
  void resetCurrentRouteLanelet(const std::shared_ptr<PlannerData> & data)
  {
    lanelet::ConstLanelet ret{};
    data->route_handler->getClosestLaneletWithinRoute(data->self_odometry->pose.pose, &ret);
    *current_route_lanelet_ = ret;
  }

  void reset()
  {
    approved_module_ptrs_.clear();
    candidate_module_ptrs_.clear();
  }

private:
  /**
   * @brief get all modules that make execution request.
   * @param decided (=approved) path.
   * @return request modules.
   */
  std::vector<SceneModulePtr> getRequestModules(
    const BehaviorModuleOutput & previous_module_output,
    const std::vector<SceneModulePtr> & deleted_modules) const;

  /**
   * @brief get manager pointer from module name.
   * @param module pointer.
   * @return manager pointer.
   */
  SceneModuleManagerPtr getManager(const SceneModulePtr & module_ptr) const
  {
    const auto itr = std::find_if(
      manager_ptrs_.begin(), manager_ptrs_.end(),
      [&module_ptr](const auto & m) { return m->name() == module_ptr->name(); });

    if (itr == manager_ptrs_.end()) {
      throw std::domain_error("unknown manager name.");
    }

    return *itr;
  }

  /**
   * @brief select a module that should be execute at first.
   * @param modules that make execution request.
   * @return the highest priority module.
   */
  SceneModulePtr selectHighestPriorityModule(std::vector<SceneModulePtr> & request_modules) const;

  /**
   * @brief swap the modules order based on it's priority.
   * @param modules.
   * @details for now, the priority is decided in config file and doesn't change runtime.
   */
  void sortByPriority(std::vector<SceneModulePtr> & modules) const
  {
    // NOTE(Mamoru Sobue): SceneModuleManagerPtr->name() == SceneModulePtr->name()
    std::sort(modules.begin(), modules.end(), [this](auto a, auto b) {
      const auto a_it = module_priorities_.find(a->name());
      const auto b_it = module_priorities_.find(b->name());
      if (a_it == module_priorities_.end() || b_it == module_priorities_.end()) {
        throw std::domain_error("unknown module name");
      }
      return a_it->second < b_it->second;
    });
  }

  /**
   * @brief stop and unregister the module from manager.
   * @param module.
   */
  void deleteExpiredModules(SceneModulePtr & module_ptr) const
  {
    module_ptr->onExit();
    module_ptr->publishObjectsOfInterestMarker();
    module_ptr.reset();
  }

  /**
   * @brief stop and remove all modules in candidate_module_ptrs_.
   */
  void clearCandidateModules()
  {
    std::for_each(candidate_module_ptrs_.begin(), candidate_module_ptrs_.end(), [this](auto & m) {
      deleteExpiredModules(m);
    });
    candidate_module_ptrs_.clear();
  }

  /**
   * @brief stop and remove all modules in approved_module_ptrs_.
   */
  void clearApprovedModules();

  /**
   * @brief update candidate_module_ptrs_ based on latest request modules.
   * @param the highest priority module in latest request modules.
   */
  void updateCandidateModules(
    const std::vector<SceneModulePtr> & request_modules,
    const SceneModulePtr & highest_priority_module);

  /**
   * @brief run the module and publish RTC status.
   * @param module.
   * @param planner data.
   * @return planning result.
   */
  BehaviorModuleOutput run(
    const SceneModulePtr & module_ptr, const std::shared_ptr<PlannerData> & planner_data,
    const BehaviorModuleOutput & previous_module_output) const;

  /**
   * @brief checks whether a path of trajectory has forward driving direction
   * @param modules that make execution request.
   * @param planner data.
   * @param decided (=approved) path.
   * @return the highest priority module in request modules, and it's planning result.
   */
  std::pair<SceneModulePtr, BehaviorModuleOutput> runRequestModules(
    const std::vector<SceneModulePtr> & request_modules, const std::shared_ptr<PlannerData> & data,
    const BehaviorModuleOutput & previous_module_output);

  /**
   * @brief run all modules in approved_module_ptrs_ and get a planning result as
   * approved_modules_output.
   * @param planner data.
   * @return valid planning result.
   * @details in this function, expired modules (ModuleStatus::FAILURE or ModuleStatus::SUCCESS) are
   * removed from approved_module_ptrs_.
   */
  SlotOutput runApprovedModules(
    const std::shared_ptr<PlannerData> & data, const BehaviorModuleOutput & upstream_slot_output,
    std::vector<SceneModulePtr> & deleted_modules);

  /**
   * @brief push back to approved_module_ptrs_.
   * @param approved module pointer.
   */
  void addApprovedModule(const SceneModulePtr & module_ptr)
  {
    approved_module_ptrs_.push_back(module_ptr);
  }

  bool isAnyCandidateExclusive() const
  {
    return std::any_of(
      candidate_module_ptrs_.begin(), candidate_module_ptrs_.end(),
      [&](const auto & m) { return !getManager(m)->isSimultaneousExecutableAsCandidateModule(); });
  }

  std::vector<SceneModuleManagerPtr> manager_ptrs_;

  std::unordered_map<std::string, size_t> module_priorities_;

  // all the slots share the current_route_lanelet of PlannerManager, so if one of the slot changed
  // it, it is reflected to later calculations as well
  std::shared_ptr<std::optional<lanelet::ConstLanelet>> current_route_lanelet_{nullptr};

  std::unordered_map<std::string, double> & processing_time_;

  std::vector<SceneModulePtr> approved_module_ptrs_;

  std::vector<SceneModulePtr> candidate_module_ptrs_;

  ModuleUpdateInfo & debug_info_;
};

}  // namespace autoware::behavior_path_planner
