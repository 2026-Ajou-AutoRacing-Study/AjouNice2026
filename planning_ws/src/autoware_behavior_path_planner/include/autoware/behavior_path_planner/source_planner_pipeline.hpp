#pragma once
#include <autoware/behavior_path_planner/sub_planner_manager.hpp>
#include <visualization_msgs/MarkerArray.h>

namespace autoware::behavior_path_planner {
// Runtime portion of the frozen PlannerManager. Plugin loading remains with
// the ROS1 host; arbitration/reference/drivable-area bodies are source-derived.
class SourcePlannerPipeline {
public:
  SourcePlannerPipeline(ros::NodeHandle & node, std::vector<SceneModuleManagerPtr> managers,
                        const std::vector<std::vector<std::string>> & slots)
  : current_route_lanelet_(std::make_shared<std::optional<lanelet::ConstLanelet>>()),
    manager_ptrs_(std::move(managers)),
    root_reference_publisher_(node.advertise<visualization_msgs::MarkerArray>(
      "debug/root_reference_path", 1)) {
    processing_time_["total_time"] = 0.0;
    for (const auto & manager : manager_ptrs_) processing_time_[manager->name()] = 0.0;
    configureModuleSlot(slots);
  }
  BehaviorModuleOutput run(const std::shared_ptr<PlannerData> & data);
  void configureModuleSlot(const std::vector<std::vector<std::string>> & slots);
  size_t activeModuleCount() const {
    size_t count = 0;
    for (const auto & slot : planner_manager_slots_)
      count += slot.approved_modules().size() + slot.candidate_modules().size();
    return count;
  }
  void reset() {
    for (const auto & manager : manager_ptrs_) manager->reset();
    *current_route_lanelet_ = std::nullopt;
    resetProcessingTime();
    for (auto & slot : planner_manager_slots_) slot.reset();
  }
private:
  void resetCurrentRouteLanelet(const std::shared_ptr<PlannerData> & data) {
    lanelet::ConstLanelet lane;
    if (!data->route_handler->getClosestLaneletWithinRoute(data->self_odometry->pose.pose, &lane))
      throw std::runtime_error("BPP cannot initialize its current route lanelet");
    *current_route_lanelet_ = lane;
  }
  void resetProcessingTime() { for (auto & entry : processing_time_) entry.second = 0.0; }
  void updateCurrentRouteLanelet(const std::shared_ptr<PlannerData> & data, bool running);
  void generateCombinedDrivableArea(BehaviorModuleOutput &, const std::shared_ptr<PlannerData> &) const;
  BehaviorModuleOutput getReferencePath(const std::shared_ptr<PlannerData> &) const;
  void publishDebugRootReferencePath(const BehaviorModuleOutput &) const;
  std::vector<std::shared_ptr<SceneModuleStatus>> getSceneModuleStatus() const {
    std::vector<std::shared_ptr<SceneModuleStatus>> result;
    for (const auto & slot : planner_manager_slots_) {
      const auto append = [&](const auto & modules) {
        for (const auto & module : modules) {
          auto status = std::make_shared<SceneModuleStatus>(module->name());
          status->is_waiting_approval = module->isWaitingApproval();
          status->status = module->getCurrentStatus();
          result.push_back(status);
        }
      };
      append(slot.approved_modules());
      append(slot.candidate_modules());
    }
    return result;
  }
  std::shared_ptr<std::optional<lanelet::ConstLanelet>> current_route_lanelet_;
  std::vector<SceneModuleManagerPtr> manager_ptrs_;
  std::unordered_map<std::string, double> processing_time_;
  ModuleUpdateInfo debug_info_;
  std::vector<SubPlannerManager> planner_manager_slots_;
  mutable std::optional<BehaviorModuleOutput> last_valid_reference_path_;
  ros::Publisher root_reference_publisher_;
};
}  // namespace autoware::behavior_path_planner
