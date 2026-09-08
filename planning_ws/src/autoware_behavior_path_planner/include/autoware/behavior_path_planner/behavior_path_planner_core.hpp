#pragma once

#include <autoware/behavior_path_planner_common/interface/bpp_input_core.hpp>
#include <autoware/behavior_path_planner_common/interface/data_readiness.hpp>
#include <autoware/behavior_path_planner_common/interface/planner_manager_slot_core.hpp>
#include <autoware/behavior_path_planner/source_planner_pipeline.hpp>
#include <pluginlib/class_loader.hpp>
#ifdef AW149_ENABLE_LANE_CHANGE
#include <autoware/behavior_path_lane_change_module/manager.hpp>
#endif
#ifdef AW149_ENABLE_STATIC_OBSTACLE_AVOIDANCE
#include <autoware/behavior_path_static_obstacle_avoidance_module/manager.hpp>
#endif
#ifdef AW149_ENABLE_AVOIDANCE_BY_LANE_CHANGE
#include <autoware/behavior_path_avoidance_by_lane_change_module/factory.hpp>
#endif
#ifdef AW149_ENABLE_DYNAMIC_OBSTACLE_AVOIDANCE
#include <autoware/behavior_path_dynamic_obstacle_avoidance_module/manager.hpp>
#endif
#ifdef AW149_ENABLE_BIDIRECTIONAL_TRAFFIC
#include <autoware/behavior_path_bidirectional_traffic_module/manager.hpp>
#endif
#include <autoware/behavior_path_side_shift_module/manager.hpp>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{
// Node-owned execution subset: source-equivalent ingress/readiness, then the
// fixed static side-shift slot. ROS topic polling and publication stay above it.
class BehaviorPathPlannerCore
{
public:
  // The source-side manager owns parameter and provider initialization.  Do
  // not synthesize a parameter-only manager here: that was the old candidate
  // ABI and bypassed the manager's ROS1 lifecycle contract.
  explicit BehaviorPathPlannerCore(ros::NodeHandle * private_node);

  bool updateLateralOffset(const std::shared_ptr<PlannerData> & planner_data,
                           const tier4_planning_msgs::LateralOffsetConstPtr & message);
  std::optional<BehaviorModuleOutput> run(
    const std::shared_ptr<PlannerData> & planner_data, const BehaviorModuleOutput & input,
    const BppInputState & input_state, double now_seconds, double cyclic_timeout_seconds,
    bool enable_traffic_signal_timeout);
  // Provider-owned markers expose scene-module request and safety decisions.
  // Keep their publication at the ROS node boundary, after a slot cycle has
  // updated the provider state.
  void publishProviderDiagnostics() const;
  const BppInputReadinessReport & lastReadinessReport() const { return last_report_; }
  size_t activeModuleCount() const
  { return configured_plugin_slot_ ? configured_plugin_slot_->activeModuleCount() : slot_.activeModuleCount(); }
  bool hasConfiguredPluginSlot() const { return static_cast<bool>(configured_plugin_slot_); }
  void resetConfiguredPlugins() { if (configured_plugin_slot_) configured_plugin_slot_->reset(); }
  const std::vector<std::string> & loadedPluginManagerNames() const
  {
    return loaded_plugin_manager_names_;
  }
  bool isInitialized() const { return initialized_; }
#ifdef AW149_ENABLE_AVOIDANCE_BY_LANE_CHANGE
  bool avoidanceByLaneChangeEnabled() const { return avoidance_by_lane_change_enabled_; }
#endif
#ifdef AW149_ENABLE_DYNAMIC_OBSTACLE_AVOIDANCE
  bool dynamicObstacleAvoidanceEnabled() const { return dynamic_obstacle_avoidance_enabled_; }
#endif
#ifdef AW149_ENABLE_STATIC_OBSTACLE_AVOIDANCE
  bool staticObstacleAvoidanceEnabled() const { return static_obstacle_avoidance_enabled_; }
#endif
#ifdef AW149_ENABLE_BIDIRECTIONAL_TRAFFIC
  bool bidirectionalTrafficEnabled() const { return bidirectional_traffic_enabled_; }
#endif
private:
#ifdef AW149_ENABLE_STATIC_OBSTACLE_AVOIDANCE
  static bool staticObstacleAvoidanceRequested(ros::NodeHandle * private_node);
#endif
#ifdef AW149_ENABLE_DYNAMIC_OBSTACLE_AVOIDANCE
  static bool dynamicObstacleAvoidanceRequested(ros::NodeHandle * private_node);
#endif
#ifdef AW149_ENABLE_BIDIRECTIONAL_TRAFFIC
  static bool bidirectionalTrafficRequested(ros::NodeHandle * private_node);
#endif
#ifdef AW149_ENABLE_AVOIDANCE_BY_LANE_CHANGE
  static bool avoidanceByLaneChangeRequested(ros::NodeHandle * private_node);
#endif
  void loadRequestedPlugins(ros::NodeHandle * private_node);
  void configureRequestedPluginSlot(ros::NodeHandle * private_node);
  static std::vector<std::shared_ptr<SceneModuleManagerCore>> makeStaticManagers(
    ros::NodeHandle * private_node, const std::shared_ptr<SideShiftModuleManager> & side_shift_manager
#ifdef AW149_ENABLE_LANE_CHANGE
    ,
    const std::shared_ptr<LaneChangeLeftModuleManager> & lane_change_left_manager,
    const std::shared_ptr<LaneChangeRightModuleManager> & lane_change_right_manager
#endif
#ifdef AW149_ENABLE_STATIC_OBSTACLE_AVOIDANCE
    , const std::shared_ptr<StaticObstacleAvoidanceModuleManager> & static_obstacle_avoidance_manager,
    bool static_obstacle_avoidance_enabled
#endif
#ifdef AW149_ENABLE_DYNAMIC_OBSTACLE_AVOIDANCE
    , const std::shared_ptr<DynamicObstacleAvoidanceModuleManager> & dynamic_obstacle_avoidance_manager,
    bool dynamic_obstacle_avoidance_enabled
#endif
#ifdef AW149_ENABLE_BIDIRECTIONAL_TRAFFIC
    , const std::shared_ptr<BidirectionalTrafficModuleManager> & bidirectional_traffic_manager,
    bool bidirectional_traffic_enabled
#endif
#ifdef AW149_ENABLE_AVOIDANCE_BY_LANE_CHANGE
    , const std::shared_ptr<SceneModuleManagerCore> & avoidance_by_lane_change_manager,
    bool avoidance_by_lane_change_enabled
#endif
    );

  std::shared_ptr<SideShiftModuleManager> side_shift_manager_;
#ifdef AW149_ENABLE_LANE_CHANGE
  std::shared_ptr<LaneChangeLeftModuleManager> lane_change_left_manager_;
  std::shared_ptr<LaneChangeRightModuleManager> lane_change_right_manager_;
#endif
#ifdef AW149_ENABLE_STATIC_OBSTACLE_AVOIDANCE
  std::shared_ptr<StaticObstacleAvoidanceModuleManager> static_obstacle_avoidance_manager_;
  bool static_obstacle_avoidance_requested_{false};
  bool static_obstacle_avoidance_enabled_{false};
#endif
#ifdef AW149_ENABLE_DYNAMIC_OBSTACLE_AVOIDANCE
  std::shared_ptr<DynamicObstacleAvoidanceModuleManager> dynamic_obstacle_avoidance_manager_;
  bool dynamic_obstacle_avoidance_requested_{false};
  bool dynamic_obstacle_avoidance_enabled_{false};
#endif
#ifdef AW149_ENABLE_BIDIRECTIONAL_TRAFFIC
  std::shared_ptr<BidirectionalTrafficModuleManager> bidirectional_traffic_manager_;
  bool bidirectional_traffic_requested_{false};
  bool bidirectional_traffic_enabled_{false};
#endif
#ifdef AW149_ENABLE_AVOIDANCE_BY_LANE_CHANGE
  bool avoidance_by_lane_change_requested_{false};
  std::shared_ptr<SceneModuleManagerCore> avoidance_by_lane_change_manager_;
  bool avoidance_by_lane_change_enabled_{false};
#endif
  // Keep the class loader and the instantiated plugin objects alive for the
  // lifetime of the BPP core.  The current ROS1 execution subset only has
  // explicit static slots, so loaded managers are registered here but are not
  // silently inserted into a slot that upstream configuration did not name.
  pluginlib::ClassLoader<SceneModuleManagerInterface> plugin_loader_{
    "autoware_behavior_path_planner",
    "autoware::behavior_path_planner::SceneModuleManagerInterface"};
  // New providers export against the package that owns the interface, avoiding
  // a plugin -> executable runtime edge in a host's build dependency closure.
  // Retain the legacy host export loader for existing installed providers.
  pluginlib::ClassLoader<SceneModuleManagerInterface> common_plugin_loader_{
    "autoware_behavior_path_planner_common",
    "autoware::behavior_path_planner::SceneModuleManagerInterface"};
  std::vector<boost::shared_ptr<SceneModuleManagerInterface>> loaded_plugin_managers_;
  std::vector<std::string> loaded_plugin_manager_names_;
  PlannerManagerSlotCore slot_;
  std::unique_ptr<SourcePlannerPipeline> configured_plugin_slot_;
  BppInputReadinessReport last_report_;
  bool initialized_{false};
};
}  // namespace autoware::behavior_path_planner
