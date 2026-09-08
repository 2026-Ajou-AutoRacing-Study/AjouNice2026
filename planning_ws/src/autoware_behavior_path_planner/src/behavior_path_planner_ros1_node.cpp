#include <autoware/behavior_path_planner/behavior_path_planner_ros1_node.hpp>

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <autoware/behavior_path_planner_common/utils/path_utils.hpp>
#include <autoware/behavior_path_planner_common/utils/route_utils.hpp>

namespace autoware::behavior_path_planner
{
BehaviorPathPlannerRos1Node::BehaviorPathPlannerRos1Node(ros::NodeHandle node, ros::NodeHandle private_node)
: node_(std::move(node)), private_node_(std::move(private_node)), core_(&private_node_)
{
  private_node_.param("cyclic_timeout", cyclic_timeout_seconds_, cyclic_timeout_seconds_);
  private_node_.param("enable_traffic_signal_timeout", enable_traffic_signal_timeout_, enable_traffic_signal_timeout_);
  private_node_.param("forward_path_length", planner_data_->parameters.forward_path_length, 50.0);
  private_node_.param("backward_path_length", planner_data_->parameters.backward_path_length, 10.0);
  // Static avoidance uses both VehicleInfo's footprint and legacy scalar
  // fields.  Populate them from the same ROS1 private parameter contract as
  // the rest of the planning pipeline; a default-zero vehicle must never be
  // silently used for obstacle filtering.
  const autoware::vehicle_info_utils::VehicleInfoUtils vehicle_info_utils(private_node_);
  planner_data_->parameters.vehicle_info = vehicle_info_utils.getVehicleInfo();
  planner_data_->parameters.wheel_base = planner_data_->parameters.vehicle_info.wheel_base_m;
  planner_data_->parameters.front_overhang = planner_data_->parameters.vehicle_info.front_overhang_m;
  planner_data_->parameters.rear_overhang = planner_data_->parameters.vehicle_info.rear_overhang_m;
  planner_data_->parameters.wheel_tread = planner_data_->parameters.vehicle_info.wheel_tread_m;
  planner_data_->parameters.vehicle_width = planner_data_->parameters.vehicle_info.vehicle_width_m;
  planner_data_->parameters.vehicle_length = planner_data_->parameters.vehicle_info.vehicle_length_m;
  if (core_.hasConfiguredPluginSlot()) {
    planner_data_->init_parameters(private_node_);
    planner_data_->drivable_area_expansion_parameters.init(private_node_);
  }
  planner_data_->ensureRouteHandler();
  route_subscriber_ = private_node_.subscribe("input/route", 1, &BehaviorPathPlannerRos1Node::onRoute, this);
  map_subscriber_ = private_node_.subscribe("input/vector_map", 1, &BehaviorPathPlannerRos1Node::onMap, this);
  scenario_subscriber_ = private_node_.subscribe("input/scenario", 1, &BehaviorPathPlannerRos1Node::onScenario, this);
  acceleration_subscriber_ = private_node_.subscribe("input/acceleration", 1, &BehaviorPathPlannerRos1Node::onAcceleration, this);
  operation_mode_subscriber_ = private_node_.subscribe("input/operation_mode_state", 1, &BehaviorPathPlannerRos1Node::onOperationMode, this);
  perception_subscriber_ = private_node_.subscribe("input/objects", 1, &BehaviorPathPlannerRos1Node::onPerception, this);
  odometry_subscriber_ = private_node_.subscribe("input/odometry", 1, &BehaviorPathPlannerRos1Node::onOdometry, this);
  occupancy_grid_subscriber_ = private_node_.subscribe("input/occupancy_grid", 1, &BehaviorPathPlannerRos1Node::onOccupancyGrid, this);
  costmap_subscriber_ = private_node_.subscribe<nav_msgs::OccupancyGrid>(
    "input/costmap", 1, [this](const nav_msgs::OccupancyGridConstPtr & message) {
      planner_data_->costmap = message;
    });
  lateral_offset_subscriber_ = private_node_.subscribe("input/lateral_offset", 1, &BehaviorPathPlannerRos1Node::onLateralOffset, this);
  path_publisher_ = private_node_.advertise<autoware_internal_planning_msgs::PathWithLaneId>("output/path", 1);
  timer_ = private_node_.createTimer(ros::Duration(0.05), &BehaviorPathPlannerRos1Node::onTimer, this);
}

void BehaviorPathPlannerRos1Node::onRoute(const autoware_planning_msgs::LaneletRouteConstPtr & message)
{
  route_ = message;
  route_dirty_ = true;
  input_state_.route_received = !message->segments.empty();
  ROS_INFO("BPP received route: segments=%zu", message->segments.size());
}
void BehaviorPathPlannerRos1Node::onMap(const autoware_map_msgs::LaneletMapBinConstPtr & message)
{
  map_ = message;
  map_dirty_ = true;
  input_state_.vector_map_received = true;
  ROS_INFO("BPP received vector map");
}
void BehaviorPathPlannerRos1Node::onScenario(const autoware_internal_planning_msgs::ScenarioConstPtr & message) { scenario_ = message; input_state_.scenario_received = true; }
void BehaviorPathPlannerRos1Node::onAcceleration(const geometry_msgs::AccelWithCovarianceStampedConstPtr & message) { planner_data_->self_acceleration = message; input_state_.acceleration_received = true; }
void BehaviorPathPlannerRos1Node::onOperationMode(const autoware_adapi_v1_msgs::OperationModeStateConstPtr & message) { planner_data_->operation_mode = message; input_state_.operation_mode_received = true; }
void BehaviorPathPlannerRos1Node::onPerception(const autoware_perception_msgs::PredictedObjectsConstPtr & message) { planner_data_->dynamic_object = message; input_state_.perception_received_at_seconds = ros::Time::now().toSec(); }
void BehaviorPathPlannerRos1Node::onOdometry(const nav_msgs::OdometryConstPtr & message) { planner_data_->self_odometry = message; input_state_.odometry_received_at_seconds = ros::Time::now().toSec(); }
void BehaviorPathPlannerRos1Node::onOccupancyGrid(const nav_msgs::OccupancyGridConstPtr & message) { planner_data_->occupancy_grid = message; input_state_.occupancy_grid_received_at_seconds = ros::Time::now().toSec(); }
void BehaviorPathPlannerRos1Node::onLateralOffset(const tier4_planning_msgs::LateralOffsetConstPtr & message) { core_.updateLateralOffset(planner_data_, message); }

void BehaviorPathPlannerRos1Node::updateRouteHandler()
{
  if (map_dirty_ && map_) {
    ROS_INFO("BPP RouteHandler setMap begin");
    planner_data_->route_handler->setMap(*map_);
    map_dirty_ = false;
    ROS_INFO("BPP RouteHandler setMap complete");
  }
  if (route_dirty_ && route_) {
    ROS_INFO("BPP RouteHandler setRoute begin: segments=%zu", route_->segments.size());
    const bool was_ready = planner_data_->route_handler->isHandlerReady();
    planner_data_->route_handler->setRoute(*route_);
    // Source node resets modules for a new route UUID, but preserves them for
    // a same-UUID modified-goal reroute.
    if (was_ready && planner_data_->prev_route_id &&
        route_->uuid != *planner_data_->prev_route_id) {
      core_.resetConfiguredPlugins();
      planner_data_->prev_modified_goal.reset();
    }
    route_dirty_ = false;
    ROS_INFO("BPP RouteHandler setRoute complete: ready=%d", planner_data_->route_handler->isHandlerReady());
  }
}

std::optional<BehaviorModuleOutput> BehaviorPathPlannerRos1Node::makeReferenceOutput() const
{
  if (!planner_data_->route_handler || !planner_data_->route_handler->isHandlerReady() ||
      !planner_data_->self_odometry) return std::nullopt;
  // RouteHandler::getCenterLinePath crops only by an arc origin at the first
  // lanelet in its sequence.  Feeding it all preferred lanelets therefore
  // made every ROS1 BPP output start at the route origin, independent of the
  // live ego pose.  Scene modules consequently saw stale current lanes and a
  // normal lane change could never enter its prepare-distance window.  Use
  // the existing upstream-compatible route utility: it selects the current
  // route sequence at ego pose and performs the required ego-relative crop.
  const auto lanes = utils::getCurrentLanes(planner_data_);
  if (lanes.empty()) return std::nullopt;
  BehaviorModuleOutput input;
  input.path = utils::getCenterLinePath(
    *planner_data_->route_handler, lanes, planner_data_->self_odometry->pose.pose,
    planner_data_->parameters.backward_path_length, planner_data_->parameters.forward_path_length,
    planner_data_->parameters);
  // Some compact Lanelet fixtures expose a valid route as only two stitched
  // endpoints.  Scene modules such as Static Obstacle need intermediate
  // geometry to form their detection polygons, so materialize a conservative
  // one-metre reference before dispatching the shared module slot.
  if (input.path.points.size() == 2U) {
    input.path = utils::resamplePathWithSpline(input.path, 1.0, true);
  }
  input.reference_path = input.path;
  if (input.path.points.empty()) return std::nullopt;
  return input;
}

void BehaviorPathPlannerRos1Node::onTimer(const ros::TimerEvent &)
{
  updateRouteHandler();
  if (!scenario_ || scenario_->current_scenario != autoware_internal_planning_msgs::Scenario::LANEDRIVING) {
    ROS_WARN_THROTTLE(1.0, "BPP waiting for lane-driving scenario: received=%d value=%s",
      static_cast<bool>(scenario_), scenario_ ? scenario_->current_scenario.c_str() : "<none>");
    return;
  }
  // Configured plugins now own the source reference/goal-around-path logic.
  // Do not reject it using the legacy current-lane centerline precondition.
  const auto input = core_.hasConfiguredPluginSlot()
    ? (planner_data_->route_handler && planner_data_->route_handler->isHandlerReady() &&
       planner_data_->self_odometry ? std::optional<BehaviorModuleOutput>{BehaviorModuleOutput{}}
                                    : std::nullopt)
    : makeReferenceOutput();
  if (!input) {
    ROS_WARN_THROTTLE(
      1.0, "BPP has no reference output: route_ready=%d odometry=%d",
      planner_data_->route_handler && planner_data_->route_handler->isHandlerReady(),
      static_cast<bool>(planner_data_->self_odometry));
    return;
  }
  const auto output = core_.run(planner_data_, *input, input_state_, ros::Time::now().toSec(), cyclic_timeout_seconds_, enable_traffic_signal_timeout_);
  core_.publishProviderDiagnostics();
  if (output) {
    planner_data_->prev_output_path =
      boost::make_shared<autoware_internal_planning_msgs::PathWithLaneId>(output->path);
    // Upstream records this after planning. Without it Start Planner sees the
    // same route as new on every cycle and discards its collision-check state.
    planner_data_->prev_route_id = planner_data_->route_handler->getRouteUuid();
  }
  if (!output || output->path.points.empty()) {
    const auto & readiness = core_.lastReadinessReport();
    ROS_WARN_THROTTLE(
      1.0,
      "BPP suppressed output: initialized=%d readiness=%d active_modules=%zu output_present=%d points=%zu",
      core_.isInitialized(), static_cast<int>(readiness.status), core_.activeModuleCount(),
      static_cast<bool>(output), output ? output->path.points.size() : 0U);
    return;
  }
  auto path = output->path;
  // RouteHandler generates centerline points in the vector-map frame but does
  // not populate the ROS1 message header.  Never publish an unframed path:
  // preserve the routed frame when supplied and otherwise use the RouteHandler
  // map-frame contract.
  path.header.frame_id = (route_ && !route_->header.frame_id.empty()) ? route_->header.frame_id : "map";
  path.header.stamp = ros::Time::now();
  planner_data_->prev_output_path = boost::make_shared<autoware_internal_planning_msgs::PathWithLaneId>(path);
  path_publisher_.publish(path);
}
}  // namespace autoware::behavior_path_planner
