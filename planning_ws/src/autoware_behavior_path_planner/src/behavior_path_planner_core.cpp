#include <autoware/behavior_path_planner/behavior_path_planner_core.hpp>

#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{
BehaviorPathPlannerCore::BehaviorPathPlannerCore(ros::NodeHandle * private_node)
: side_shift_manager_(std::make_shared<SideShiftModuleManager>()),
#ifdef AW149_ENABLE_LANE_CHANGE
  lane_change_left_manager_(std::make_shared<LaneChangeLeftModuleManager>()),
  lane_change_right_manager_(std::make_shared<LaneChangeRightModuleManager>()),
#endif
#ifdef AW149_ENABLE_STATIC_OBSTACLE_AVOIDANCE
  static_obstacle_avoidance_manager_(std::make_shared<StaticObstacleAvoidanceModuleManager>()),
  static_obstacle_avoidance_requested_(staticObstacleAvoidanceRequested(private_node)),
  static_obstacle_avoidance_enabled_(
    static_obstacle_avoidance_requested_ && static_obstacle_avoidance_manager_->init(private_node)),
#endif
#ifdef AW149_ENABLE_DYNAMIC_OBSTACLE_AVOIDANCE
  dynamic_obstacle_avoidance_manager_(std::make_shared<DynamicObstacleAvoidanceModuleManager>()),
  dynamic_obstacle_avoidance_requested_(dynamicObstacleAvoidanceRequested(private_node)),
  dynamic_obstacle_avoidance_enabled_(
    dynamic_obstacle_avoidance_requested_ && dynamic_obstacle_avoidance_manager_->init(private_node)),
#endif
#ifdef AW149_ENABLE_BIDIRECTIONAL_TRAFFIC
  bidirectional_traffic_manager_(std::make_shared<BidirectionalTrafficModuleManager>()),
  bidirectional_traffic_requested_(bidirectionalTrafficRequested(private_node)),
  bidirectional_traffic_enabled_(
    bidirectional_traffic_requested_ && bidirectional_traffic_manager_->init(private_node)),
#endif
#ifdef AW149_ENABLE_AVOIDANCE_BY_LANE_CHANGE
  avoidance_by_lane_change_requested_(avoidanceByLaneChangeRequested(private_node)),
  avoidance_by_lane_change_manager_(
    avoidance_by_lane_change_requested_ ? makeInitializedAvoidanceByLaneChangeManager(private_node) : nullptr),
  avoidance_by_lane_change_enabled_(static_cast<bool>(avoidance_by_lane_change_manager_)),
#endif
  slot_(makeStaticManagers(
    private_node, side_shift_manager_
#ifdef AW149_ENABLE_LANE_CHANGE
    , lane_change_left_manager_, lane_change_right_manager_
#endif
#ifdef AW149_ENABLE_STATIC_OBSTACLE_AVOIDANCE
    , static_obstacle_avoidance_manager_, static_obstacle_avoidance_enabled_
#endif
#ifdef AW149_ENABLE_DYNAMIC_OBSTACLE_AVOIDANCE
    , dynamic_obstacle_avoidance_manager_, dynamic_obstacle_avoidance_enabled_
#endif
#ifdef AW149_ENABLE_BIDIRECTIONAL_TRAFFIC
    , bidirectional_traffic_manager_, bidirectional_traffic_enabled_
#endif
#ifdef AW149_ENABLE_AVOIDANCE_BY_LANE_CHANGE
    , avoidance_by_lane_change_manager_, avoidance_by_lane_change_enabled_
#endif
    ))
{
  if (!private_node) return;
  bool enable_side_shift_module=true;
  private_node->param("enable_side_shift_module",enable_side_shift_module,true);
  initialized_=!enable_side_shift_module || side_shift_manager_->init(private_node);
  // Preserve the upstream launch contract: launch_modules names manager
  // plugins, and each requested plugin is loaded and initialized before slot
  // configuration is applied.  This candidate deliberately does not invent a
  // slot for a plugin absent from the configured static-slot subset.
  loadRequestedPlugins(private_node);
  configureRequestedPluginSlot(private_node);
  // Lane-change requires the complete ROS1 translation of the source parameter
  // file.  Keep it out of the execution slot unless that translation is
  // explicitly selected; zero-valued implicit parameters are unsafe here.
#ifdef AW149_ENABLE_LANE_CHANGE
  bool enable_lane_change_modules = false;
  private_node->param("enable_lane_change_modules", enable_lane_change_modules, false);
  if (enable_lane_change_modules) {
    lane_change_left_manager_->init(private_node);
    lane_change_right_manager_->init(private_node);
  }
#endif
#ifdef AW149_ENABLE_STATIC_OBSTACLE_AVOIDANCE
  // A requested path-altering provider without its rendered ROS1 parameters
  // fails core initialization; it must not silently degrade to SideShift.
  initialized_ = initialized_ &&
    (!static_obstacle_avoidance_requested_ || static_obstacle_avoidance_enabled_);
#endif
#ifdef AW149_ENABLE_DYNAMIC_OBSTACLE_AVOIDANCE
  // Default-disabled gate: an incomplete ROS1 parameter translation must not
  // cause a module to enter the live BPP slot.  An explicit request that
  // cannot initialize is a core initialization failure, rather than a silent
  // fallback to an invented avoidance policy.
  initialized_ = initialized_ &&
    (!dynamic_obstacle_avoidance_requested_ || dynamic_obstacle_avoidance_enabled_);
#endif
#ifdef AW149_ENABLE_BIDIRECTIONAL_TRAFFIC
  // Bidirectional traffic changes a live path only when explicitly requested.
  // An incomplete ROS1 contract is a failed core initialization, never a
  // silent omission of a requested safety behavior.
  initialized_ = initialized_ &&
    (!bidirectional_traffic_requested_ || bidirectional_traffic_enabled_);
#endif
#ifdef AW149_ENABLE_AVOIDANCE_BY_LANE_CHANGE
  initialized_ = initialized_ &&
    (!avoidance_by_lane_change_requested_ || avoidance_by_lane_change_enabled_);
#endif
}

void BehaviorPathPlannerCore::publishProviderDiagnostics() const
{
  for (const auto & manager : loaded_plugin_managers_) {
    // SourcePlannerPipeline publishes RTC/factors once after arbitration.
    // Publishing factors again flushes an empty array over the real result.
    if (!configured_plugin_slot_) {
      manager->updateObserver();
      manager->publishRTCStatus();
      manager->publishPlanningFactors();
    }
    manager->publishMarkers();
  }
#ifdef AW149_ENABLE_LANE_CHANGE
  if (lane_change_left_manager_) lane_change_left_manager_->publishMarkers();
  if (lane_change_right_manager_) lane_change_right_manager_->publishMarkers();
#endif
}

void BehaviorPathPlannerCore::loadRequestedPlugins(ros::NodeHandle * private_node)
{
  if (!private_node) return;
  std::vector<std::string> requested;
  if (!private_node->getParam("launch_modules", requested)) return;
  for (const auto & class_name : requested) {
    if (class_name.empty()) continue;
    auto & loader = common_plugin_loader_.isClassAvailable(class_name)
                     ? common_plugin_loader_ : plugin_loader_;
    if (!loader.isClassAvailable(class_name)) {
      throw std::runtime_error("requested BPP manager plugin is unavailable: " + class_name);
    }
    const auto manager = loader.createInstance(class_name);
    if (!manager) throw std::runtime_error("requested BPP manager plugin is null: " + class_name);
    if (!manager->initialize(private_node)) {
      throw std::runtime_error("requested BPP manager plugin initialization failed: " + class_name);
    }
    for (const auto & loaded : loaded_plugin_managers_) {
      if (loaded && loaded->name() == manager->name()) {
        throw std::runtime_error("duplicate BPP manager plugin: " + manager->name());
      }
    }
    loaded_plugin_manager_names_.push_back(manager->name());
    loaded_plugin_managers_.push_back(manager);
  }
}

void BehaviorPathPlannerCore::configureRequestedPluginSlot(ros::NodeHandle * node)
{
  if (loaded_plugin_managers_.empty()) return;
  std::vector<std::string> slots;
  if (!node->getParam("slots", slots))
    throw std::runtime_error("BPP launch_modules requires explicit source slots configuration");
  std::vector<SceneModuleManagerPtr> selected;
  std::vector<std::vector<std::string>> configuration;
  std::vector<std::string> used;
  for (const auto & slot_name : slots) {
    std::vector<std::string> names;
    if (!node->getParam(slot_name, names))
      throw std::runtime_error("Missing BPP slot configuration: " + slot_name);
    configuration.push_back(names);
    for (const auto & name : names) {
      for (const auto & manager : loaded_plugin_managers_) {
        if (manager->name() != name) continue;
        if (std::find(used.begin(), used.end(), name) != used.end())
          throw std::runtime_error("Duplicate BPP slot module: " + name);
        used.push_back(name);
        selected.emplace_back(manager.get(), [manager](SceneModuleManagerInterface *) {});
      }
    }
  }
  if (selected.size() != loaded_plugin_managers_.size())
    throw std::runtime_error("A requested BPP plugin has no configured execution slot");
  configured_plugin_slot_ = std::make_unique<SourcePlannerPipeline>(
    *node, std::move(selected), configuration);
}

#ifdef AW149_ENABLE_STATIC_OBSTACLE_AVOIDANCE
bool BehaviorPathPlannerCore::staticObstacleAvoidanceRequested(ros::NodeHandle * private_node)
{
  bool enabled = false;
  if (private_node) private_node->param("enable_static_obstacle_avoidance_module", enabled, false);
  return enabled;
}
#endif

#ifdef AW149_ENABLE_DYNAMIC_OBSTACLE_AVOIDANCE
bool BehaviorPathPlannerCore::dynamicObstacleAvoidanceRequested(ros::NodeHandle * private_node)
{
  bool enabled = false;
  if (private_node) {
    private_node->param("enable_dynamic_obstacle_avoidance_module", enabled, false);
  }
  return enabled;
}
#endif

#ifdef AW149_ENABLE_AVOIDANCE_BY_LANE_CHANGE
bool BehaviorPathPlannerCore::avoidanceByLaneChangeRequested(ros::NodeHandle * private_node)
{
  bool enabled = false;
  if (private_node) private_node->param("enable_avoidance_by_lane_change_module", enabled, false);
  return enabled;
}
#endif

#ifdef AW149_ENABLE_BIDIRECTIONAL_TRAFFIC
bool BehaviorPathPlannerCore::bidirectionalTrafficRequested(ros::NodeHandle * private_node)
{
  bool enabled = false;
  if (private_node) private_node->param("enable_bidirectional_traffic_module", enabled, false);
  return enabled;
}
#endif

std::vector<std::shared_ptr<SceneModuleManagerCore>> BehaviorPathPlannerCore::makeStaticManagers(
  ros::NodeHandle * private_node, const std::shared_ptr<SideShiftModuleManager> & side_shift_manager
#ifdef AW149_ENABLE_LANE_CHANGE
  ,
  const std::shared_ptr<LaneChangeLeftModuleManager> & lane_change_left_manager,
  const std::shared_ptr<LaneChangeRightModuleManager> & lane_change_right_manager
#endif
#ifdef AW149_ENABLE_STATIC_OBSTACLE_AVOIDANCE
  , const std::shared_ptr<StaticObstacleAvoidanceModuleManager> & static_obstacle_avoidance_manager,
  const bool static_obstacle_avoidance_enabled
#endif
#ifdef AW149_ENABLE_DYNAMIC_OBSTACLE_AVOIDANCE
  , const std::shared_ptr<DynamicObstacleAvoidanceModuleManager> & dynamic_obstacle_avoidance_manager,
  const bool dynamic_obstacle_avoidance_enabled
#endif
#ifdef AW149_ENABLE_BIDIRECTIONAL_TRAFFIC
  , const std::shared_ptr<BidirectionalTrafficModuleManager> & bidirectional_traffic_manager,
  const bool bidirectional_traffic_enabled
#endif
#ifdef AW149_ENABLE_AVOIDANCE_BY_LANE_CHANGE
  , const std::shared_ptr<SceneModuleManagerCore> & avoidance_by_lane_change_manager,
  const bool avoidance_by_lane_change_enabled
#endif
  )
{
  // SideShift is an independent lateral-offset feature.  A focused
  // lane-change runtime may exclude it explicitly; otherwise preserve the
  // existing default host composition.  This is a module-selection gate, not
  // a fallback: a requested SideShift remains present and is independently
  // validated by its own runtime gate.
  bool enable_side_shift_module = true;
  if (private_node) {
    private_node->param("enable_side_shift_module", enable_side_shift_module, true);
  }
  std::vector<std::shared_ptr<SceneModuleManagerCore>> managers;
  if (enable_side_shift_module) managers.push_back(side_shift_manager);
#ifdef AW149_ENABLE_STATIC_OBSTACLE_AVOIDANCE
  if (static_obstacle_avoidance_enabled) {
    managers.push_back(static_obstacle_avoidance_manager);
  }
#endif
#ifdef AW149_ENABLE_LANE_CHANGE
  bool enable_lane_change_modules = false;
  if (private_node) private_node->param("enable_lane_change_modules", enable_lane_change_modules, false);
  if (enable_lane_change_modules) {
    managers.push_back(lane_change_left_manager);
    managers.push_back(lane_change_right_manager);
  }
#endif
#ifdef AW149_ENABLE_DYNAMIC_OBSTACLE_AVOIDANCE
  (void)private_node;
  if (dynamic_obstacle_avoidance_enabled) {
    managers.push_back(dynamic_obstacle_avoidance_manager);
  }
#endif
#ifdef AW149_ENABLE_BIDIRECTIONAL_TRAFFIC
  if (bidirectional_traffic_enabled) {
    managers.push_back(bidirectional_traffic_manager);
  }
#endif
#ifdef AW149_ENABLE_AVOIDANCE_BY_LANE_CHANGE
  if (avoidance_by_lane_change_enabled) {
    managers.push_back(avoidance_by_lane_change_manager);
  }
#endif
  return managers;
}

bool BehaviorPathPlannerCore::updateLateralOffset(
  const std::shared_ptr<PlannerData> & planner_data,
  const tier4_planning_msgs::LateralOffsetConstPtr & message)
{
  return BppInputCore::updateLateralOffset(planner_data, message);
}

std::optional<BehaviorModuleOutput> BehaviorPathPlannerCore::run(
  const std::shared_ptr<PlannerData> & planner_data, const BehaviorModuleOutput & input,
  const BppInputState & input_state, const double now_seconds,
  const double cyclic_timeout_seconds, const bool enable_traffic_signal_timeout)
{
  last_report_ = evaluateBppInputReadiness(
    input_state, now_seconds, cyclic_timeout_seconds, enable_traffic_signal_timeout);
  if (!initialized_ || !last_report_.canRunPlanner() || !planner_data) return std::nullopt;
#ifdef AW149_ENABLE_DYNAMIC_OBSTACLE_AVOIDANCE
  // A received timestamp alone is not a valid perception input.  The DOA
  // module consumes the object message itself, so never enter its slot with a
  // null pointer; an explicitly empty PredictedObjects message remains valid.
  if (dynamic_obstacle_avoidance_enabled_ && !planner_data->dynamic_object) return std::nullopt;
  // Likewise, an empty upstream BPP path has no geometry on which DOA may
  // compute obstacle polygons.  Preserve the upstream empty output instead
  // of manufacturing a path or calling a geometry routine with no points.
  if (dynamic_obstacle_avoidance_enabled_ && input.path.points.empty()) return input;
#endif
#ifdef AW149_ENABLE_STATIC_OBSTACLE_AVOIDANCE
  // Static obstacle avoidance dereferences perception, route, and current
  // acceleration while constructing avoidance data.  A freshness timestamp
  // is not a substitute for those payloads.  Keep the ROS1 port fail-closed,
  // as the upstream module cannot safely infer any absent scene context.  An
  // explicitly empty object array remains a valid scene input and lets the
  // module return its ordinary no-request output.
  if (static_obstacle_avoidance_enabled_ &&
      (!planner_data->dynamic_object || !planner_data->route_handler ||
       !planner_data->self_acceleration)) {
    return std::nullopt;
  }
  // The scene module derives lane and object geometry from the reference
  // path.  Preserve an empty upstream path rather than invoking it with no
  // geometry to process.
  if (static_obstacle_avoidance_enabled_ && input.path.points.empty()) return input;
#endif
#ifdef AW149_ENABLE_BIDIRECTIONAL_TRAFFIC
  // The module consumes perception and the route map.  Do not enter its slot
  // with a merely timestamped object stream or a missing route provider.
  if (bidirectional_traffic_enabled_ && (!planner_data->dynamic_object || !planner_data->route_handler)) {
    return std::nullopt;
  }
  if (bidirectional_traffic_enabled_ && input.path.points.empty()) return input;
#endif
  if (configured_plugin_slot_) return configured_plugin_slot_->run(planner_data);
  return slot_.run(planner_data, input);
}
}  // namespace autoware::behavior_path_planner
