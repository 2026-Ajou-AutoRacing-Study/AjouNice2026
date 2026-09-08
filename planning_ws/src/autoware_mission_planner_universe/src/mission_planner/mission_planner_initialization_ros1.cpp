#include "mission_planner_initialization_ros1.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/lanelet2_utils/nn_search.hpp>
#include <ros/package.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::mission_planner_universe
{
MissionPlannerInitializationRos1::MissionPlannerInitializationRos1(
  ros::NodeHandle &, ros::NodeHandle & private_node)
: private_node_(private_node),
  plugin_loader_(
    "autoware_mission_planner_universe", "autoware::mission_planner_universe::PlannerPlugin",
    "plugin", {ros::package::getPath("autoware_mission_planner_universe") +
               "/plugins/plugin_description.xml"}),
  tf_buffer_(ros::Duration(10.0)),
  tf_listener_(tf_buffer_),
  arrival_checker_(&private_node_)
{
  if (ros::package::getPath("autoware_mission_planner_universe").empty()) {
    throw std::runtime_error("autoware_mission_planner_universe is not discoverable by rospack");
  }
  private_node_.param("map_frame", map_frame_, std::string("map"));
  private_node_.param(
    "allow_reroute_in_autonomous_mode", allow_reroute_in_autonomous_mode_, false);
  private_node_.param("reroute_time_threshold", reroute_time_threshold_, 10.0);
  private_node_.param("minimum_reroute_length", minimum_reroute_length_, 30.0);
  private_node_.param("goal_lanelet_transparency", goal_lanelet_transparency_, 0.05);
  try {
    planner_ = plugin_loader_.createInstance(
      "autoware::mission_planner_universe::lanelet2::DefaultPlanner");
  } catch (const std::exception & error) {
    throw std::runtime_error(std::string("DefaultPlanner plugin creation failed: ") + error.what());
  }
  try {
    planner_->initialize(&private_node_);
  } catch (const std::exception & error) {
    throw std::runtime_error(std::string("DefaultPlanner initialization failed: ") + error.what());
  }
  map_subscriber_ = private_node_.subscribe(
    "input/vector_map", 10, &MissionPlannerInitializationRos1::on_map, this);
  odometry_subscriber_ = private_node_.subscribe(
    "input/odometry", 1, &MissionPlannerInitializationRos1::on_odometry, this);
  modified_goal_subscriber_ = private_node_.subscribe(
    "input/modified_goal", 1, &MissionPlannerInitializationRos1::on_modified_goal, this);
  operation_mode_subscriber_ = private_node_.subscribe(
    "input/operation_mode_state", 1, &MissionPlannerInitializationRos1::on_operation_mode, this);
  reroute_availability_subscriber_ = private_node_.subscribe(
    "input/reroute_availability", 1,
    &MissionPlannerInitializationRos1::on_reroute_availability, this);
  state_publisher_ = private_node_.advertise<autoware_planning_msgs::RouteState>("state", 1, true);
  route_publisher_ = private_node_.advertise<autoware_planning_msgs::LaneletRoute>("route", 1, true);
  route_marker_publisher_ = private_node_.advertise<visualization_msgs::MarkerArray>(
    "debug/route_marker", 1, true);
  clear_route_service_ = private_node_.advertiseService(
    "clear_route", &MissionPlannerInitializationRos1::on_clear_route, this);
  set_lanelet_route_service_ = private_node_.advertiseService(
    "set_lanelet_route", &MissionPlannerInitializationRos1::on_set_lanelet_route, this);
  set_preferred_primitive_service_ = private_node_.advertiseService(
    "set_preferred_primitive", &MissionPlannerInitializationRos1::on_set_preferred_primitive, this);
  set_waypoint_route_service_ = private_node_.advertiseService(
    "set_waypoint_route", &MissionPlannerInitializationRos1::on_set_waypoint_route, this);
}

void MissionPlannerInitializationRos1::on_map(
  const autoware_map_msgs::LaneletMapBinConstPtr & message)
{
  try {
    lanelet_map_ = autoware::experimental::lanelet2_utils::remove_const(
      autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*message));
    map_received_ = static_cast<bool>(lanelet_map_);
  } catch (const std::exception & error) {
    lanelet_map_.reset();
    map_received_ = false;
    ROS_ERROR_NAMED("mission_planner", "Lanelet map deserialization failed: %s", error.what());
  }
  publish_ready_if_complete();
}

void MissionPlannerInitializationRos1::on_odometry(const nav_msgs::OdometryConstPtr & message)
{
  latest_odometry_ = *message;
  odometry_received_ = true;
  publish_ready_if_complete();
  if (route_state_ == autoware_planning_msgs::RouteState::SET) {
    geometry_msgs::PoseStamped pose;
    pose.header = message->header;
    pose.pose = message->pose.pose;
    if (arrival_checker_.is_arrived(pose)) {
      publish_state(autoware_planning_msgs::RouteState::ARRIVED);
    }
  }
}

void MissionPlannerInitializationRos1::on_modified_goal(
  const autoware_planning_msgs::PoseWithUuidStampedConstPtr & message)
{
  if (!ready_published_ || route_state_ != autoware_planning_msgs::RouteState::SET ||
      current_route_.segments.empty() || message->uuid != current_route_.uuid) {
    ROS_WARN_NAMED("mission_planner", "Ignoring modified goal without a matching active route");
    return;
  }
  geometry_msgs::Pose goal_pose;
  if (!transform_goal_to_map(message->pose, message->header, &goal_pose)) {
    ROS_WARN_NAMED("mission_planner", "Ignoring modified goal that cannot be transformed to map");
    return;
  }
  publish_state(autoware_planning_msgs::RouteState::REROUTING);
  PlannerPlugin::RoutePoints points{latest_odometry_.pose.pose, goal_pose};
  auto route = planner_->plan(points);
  if (route.segments.empty()) {
    ROS_WARN_NAMED("mission_planner", "Modified goal produced an empty route");
    publish_state(autoware_planning_msgs::RouteState::SET);
    return;
  }
  route.header = message->header;
  route.header.frame_id = map_frame_;
  route.uuid = message->uuid;
  route.allow_modification = current_route_.allow_modification;
  current_route_ = route;
  original_route_.reset();
  planner_->updateRoute(current_route_);
  autoware_planning_msgs::PoseWithUuidStamped goal;
  goal.header = current_route_.header;
  goal.pose = current_route_.goal_pose;
  goal.uuid = current_route_.uuid;
  arrival_checker_.set_goal(goal);
  publish_route_outputs();
  publish_state(autoware_planning_msgs::RouteState::SET);
}

void MissionPlannerInitializationRos1::on_operation_mode(
  const autoware_adapi_v1_msgs::OperationModeStateConstPtr & message)
{
  operation_mode_state_ = message;
}

void MissionPlannerInitializationRos1::on_reroute_availability(
  const tier4_planning_msgs::RerouteAvailabilityConstPtr & message)
{
  reroute_availability_ = message;
}

void MissionPlannerInitializationRos1::publish_ready_if_complete()
{
  if (ready_published_ || !map_received_ || !odometry_received_ || !planner_->ready()) {
    return;
  }
  publish_state(autoware_planning_msgs::RouteState::UNSET);
  ready_published_ = true;
  ROS_INFO_NAMED("mission_planner", "MissionPlanner initialization complete; Route API state is UNSET.");
}

void MissionPlannerInitializationRos1::publish_state(uint8_t state_value)
{
  autoware_planning_msgs::RouteState state;
  const auto now = ros::Time::now();
  state.stamp.sec = now.sec;
  state.stamp.nanosec = now.nsec;
  state.state = state_value;
  route_state_ = state_value;
  state_publisher_.publish(state);
}

bool MissionPlannerInitializationRos1::transform_goal_to_map(
  const geometry_msgs::Pose & input, const std_msgs::Header & header, geometry_msgs::Pose * output)
{
  if (header.frame_id.empty() || header.frame_id == map_frame_) {
    *output = input;
    return true;
  }
  geometry_msgs::PoseStamped source;
  source.header = header;
  source.pose = input;
  try {
    geometry_msgs::PoseStamped transformed;
    tf_buffer_.transform(source, transformed, map_frame_, ros::Duration(1.0));
    *output = transformed.pose;
    return true;
  } catch (const tf2::TransformException & error) {
    ROS_WARN_NAMED("mission_planner", "Goal transform to %s failed: %s", map_frame_.c_str(), error.what());
    return false;
  }
}

bool MissionPlannerInitializationRos1::is_autonomous_control_active() const
{
  return operation_mode_state_ &&
         operation_mode_state_->mode == autoware_adapi_v1_msgs::OperationModeState::AUTONOMOUS &&
         operation_mode_state_->is_autoware_control_enabled;
}

bool MissionPlannerInitializationRos1::check_reroute_safety(
  const autoware_planning_msgs::LaneletRoute & original_route,
  const autoware_planning_msgs::LaneletRoute & target_route) const
{
  if (original_route.segments.empty() || target_route.segments.empty() || !lanelet_map_ || !odometry_received_) {
    ROS_ERROR_NAMED("mission_planner", "Check reroute safety failed: route, map, or odometry is unset");
    return false;
  }
  const auto current_velocity = latest_odometry_.twist.twist.linear.x;
  if (current_velocity < 0.01) return true;
  const auto same = [](const auto & a, const auto & b) {
    return a.size() == b.size() && std::all_of(a.begin(), a.end(), [&](const auto & p) {
      return std::any_of(b.begin(), b.end(), [&](const auto & q) { return p.id == q.id; });
    });
  };
  std::optional<std::pair<size_t, size_t>> start;
  for (size_t i = 0; i < original_route.segments.size() && !start; ++i)
    for (size_t j = 0; j < target_route.segments.size(); ++j)
      if (same(original_route.segments.at(i).primitives, target_route.segments.at(j).primitives)) { start = {i, j}; break; }
  if (!start) { ROS_ERROR_NAMED("mission_planner", "Check reroute safety failed: no common route segment"); return false; }
  const auto on_first = std::any_of(target_route.segments.front().primitives.begin(), target_route.segments.front().primitives.end(), [&](const auto & p) {
    return autoware::experimental::lanelet2_utils::is_in_lanelet(target_route.start_pose, lanelet_map_->laneletLayer.get(p.id));
  });
  if (!on_first) { ROS_ERROR_NAMED("mission_planner", "Check reroute safety failed: ego is outside target start section"); return false; }
  const auto [oi, ti] = *start;
  size_t oe = oi, te = ti;
  for (size_t n = 1; ti + n < target_route.segments.size() && oi + n < original_route.segments.size() && same(original_route.segments.at(oi+n).primitives, target_route.segments.at(ti+n).primitives); ++n) { oe = oi+n; te = ti+n; }
  const auto remaining = [&](const auto & ps, const geometry_msgs::Pose & pose) -> std::optional<double> {
    lanelet::ConstLanelets ls; for (const auto & p : ps) ls.push_back(lanelet_map_->laneletLayer.get(p.id));
    const auto closest = autoware::experimental::lanelet2_utils::get_closest_lanelet(ls, pose);
    if (!closest) return {};
    const auto arc = lanelet::geometry::toArcCoordinates(lanelet::utils::to2D(closest->centerline()), lanelet::utils::to2D(autoware::experimental::lanelet2_utils::from_ros(pose.position)).basicPoint());
    return lanelet::geometry::length2d(*closest) - arc.length;
  };
  const auto initial = remaining(original_route.segments.at(ti != 0 && oi > 1 ? oi - 1 : oi).primitives, target_route.start_pose);
  if (!initial) { ROS_ERROR_NAMED("mission_planner", "Check reroute safety failed: no closest common lanelet"); return false; }
  double length = *initial;
  for (size_t i = oi + 1; i <= oe; ++i) { const auto & ps = original_route.segments.at(i).primitives; if (ps.empty()) break; std::vector<double> lengths; for (const auto & p : ps) lengths.push_back(lanelet::geometry::length2d(lanelet_map_->laneletLayer.get(p.id))); length += *std::min_element(lengths.begin(), lengths.end()); }
  for (const auto & p : target_route.segments.at(te).primitives) { const auto l = lanelet_map_->laneletLayer.get(p.id); if (autoware::experimental::lanelet2_utils::is_in_lanelet(target_route.goal_pose, l)) { const auto arc = lanelet::geometry::toArcCoordinates(lanelet::utils::to2D(l.centerline()), lanelet::utils::to2D(autoware::experimental::lanelet2_utils::from_ros(target_route.goal_pose.position)).basicPoint()); length = std::max(length - (lanelet::geometry::length2d(l) - arc.length), 0.0); break; } }
  const auto safety = std::max(current_velocity * reroute_time_threshold_, minimum_reroute_length_);
  if (length > safety) return true;
  ROS_WARN_NAMED("mission_planner", "Reroute rejected: common length %.3f <= safety length %.3f", length, safety);
  return false;
}

void MissionPlannerInitializationRos1::publish_route_outputs()
{
  route_publisher_.publish(current_route_);
  route_marker_publisher_.publish(planner_->visualize(current_route_, static_cast<float>(goal_lanelet_transparency_)));
}

bool MissionPlannerInitializationRos1::on_clear_route(
  autoware_planning_msgs::ClearRoute::Request &, autoware_planning_msgs::ClearRoute::Response & response)
{
  if (!ready_published_) {
    response.status.success = false;
    response.status.code = autoware_common_msgs::ResponseStatus::SERVICE_UNREADY;
    response.status.message = "MissionPlanner is not ready";
    return true;
  }
  current_route_ = autoware_planning_msgs::LaneletRoute{};
  original_route_.reset();
  arrival_checker_.set_goal();
  planner_->clearRoute();
  publish_state(autoware_planning_msgs::RouteState::UNSET);
  response.status.success = true;
  response.status.message = "Route cleared";
  return true;
}

bool MissionPlannerInitializationRos1::on_set_lanelet_route(
  autoware_planning_msgs::SetLaneletRoute::Request & request,
  autoware_planning_msgs::SetLaneletRoute::Response & response)
{
  if (!ready_published_) {
    response.status.success = false;
    response.status.code = autoware_common_msgs::ResponseStatus::SERVICE_UNREADY;
    response.status.message = "MissionPlanner is not ready";
    return true;
  }
  const bool is_reroute =
    route_state_ == autoware_planning_msgs::RouteState::SET && !current_route_.segments.empty();
  if (is_reroute && !operation_mode_state_) {
    response.status.success = false;
    response.status.code = autoware_common_msgs::ResponseStatus::SERVICE_UNREADY;
    response.status.message = "Operation mode state is not received";
    return true;
  }
  const bool is_autonomous_driving = is_autonomous_control_active();
  if (is_reroute && is_autonomous_driving) {
    if (!allow_reroute_in_autonomous_mode_) {
      response.status.success = false;
      response.status.code = autoware_common_msgs::ResponseStatus::NO_EFFECT;
      response.status.message = "Reroute is not allowed in autonomous mode";
      return true;
    }
    if (!reroute_availability_ || !reroute_availability_->availability) {
      response.status.success = false;
      response.status.code = autoware_common_msgs::ResponseStatus::NO_EFFECT;
      response.status.message = "Cannot reroute as the planner is not in lane following";
      return true;
    }
  }
  if (request.segments.empty()) {
    response.status.success = false;
    response.status.message = "Route segments are empty";
    return true;
  }
  geometry_msgs::Pose goal_pose;
  if (!transform_goal_to_map(request.goal_pose, request.header, &goal_pose)) {
    response.status.success = false;
    response.status.code = autoware_common_msgs::ResponseStatus::TRANSFORM_ERROR;
    response.status.message = "Goal pose cannot be transformed into the map frame";
    return true;
  }
  autoware_planning_msgs::LaneletRoute target_route;
  target_route.header = request.header;
  target_route.header.frame_id = map_frame_;
  target_route.start_pose = latest_odometry_.pose.pose;
  target_route.goal_pose = goal_pose;
  target_route.segments = request.segments;
  target_route.uuid = request.uuid;
  target_route.allow_modification = request.allow_modification;
  if (is_reroute && is_autonomous_driving && !check_reroute_safety(current_route_, target_route)) {
    response.status.success = false;
    response.status.code = autoware_common_msgs::ResponseStatus::NO_EFFECT;
    response.status.message = "New route is not safe. Reroute failed.";
    return true;
  }
  current_route_ = target_route;
  original_route_.reset();
  planner_->updateRoute(current_route_);
  autoware_planning_msgs::PoseWithUuidStamped goal;
  goal.header = current_route_.header;
  goal.pose = current_route_.goal_pose;
  goal.uuid = current_route_.uuid;
  arrival_checker_.set_goal(goal);
  publish_route_outputs();
  publish_state(autoware_planning_msgs::RouteState::SET);
  response.status.success = true;
  response.status.message = "Lanelet route set";
  return true;
}

bool MissionPlannerInitializationRos1::on_set_preferred_primitive(
  autoware_planning_msgs::SetPreferredPrimitive::Request & request,
  autoware_planning_msgs::SetPreferredPrimitive::Response & response)
{
  if (!ready_published_) {
    response.status.success = false;
    response.status.code = autoware_common_msgs::ResponseStatus::SERVICE_UNREADY;
    response.status.message = "MissionPlanner is not ready";
    return true;
  }
  if (current_route_.segments.empty()) {
    response.status.success = false;
    response.status.message = "The route has not been set yet";
    return true;
  }
  if (request.uuid != current_route_.uuid) {
    response.status.success = false;
    response.status.message = "Route UUID does not match the current route";
    return true;
  }
  if (request.reset) {
    if (!original_route_) {
      response.status.success = false;
      response.status.message = "There is no saved original route to reset to";
      return true;
    }
    current_route_ = *original_route_;
    original_route_.reset();
    planner_->updateRoute(current_route_);
    publish_route_outputs();
    response.status.success = true;
    response.status.message = "Successfully reset preferred primitive";
    return true;
  }
  if (request.preferred_primitives.size() != current_route_.segments.size()) {
    response.status.success = false;
    response.status.message = "The number of preferred primitives does not match the route segments";
    return true;
  }
  auto updated_route = current_route_;
  for (size_t index = 0; index < updated_route.segments.size(); ++index) {
    const auto & preferred = request.preferred_primitives.at(index);
    const auto & primitives = updated_route.segments.at(index).primitives;
    const auto found = std::any_of(
      primitives.begin(), primitives.end(), [&preferred](const auto & primitive) {
        return primitive.id == preferred.id;
      });
    if (!found) {
      response.status.success = false;
      response.status.message = "Preferred primitive does not belong to its lanelet segment";
      return true;
    }
    updated_route.segments.at(index).preferred_primitive = preferred;
  }
  if (!original_route_) {
    original_route_ = current_route_;
  }
  current_route_ = updated_route;
  planner_->updateRoute(current_route_);
  publish_route_outputs();
  response.status.success = true;
  response.status.message = "Successfully set preferred primitive";
  return true;
}

bool MissionPlannerInitializationRos1::on_set_waypoint_route(
  autoware_planning_msgs::SetWaypointRoute::Request & request,
  autoware_planning_msgs::SetWaypointRoute::Response & response)
{
  if (!ready_published_) {
    response.status.success = false;
    response.status.code = autoware_common_msgs::ResponseStatus::SERVICE_UNREADY;
    response.status.message = "MissionPlanner is not ready";
    return true;
  }
  const bool is_reroute =
    route_state_ == autoware_planning_msgs::RouteState::SET && !current_route_.segments.empty();
  if (is_reroute && !operation_mode_state_) {
    response.status.success = false;
    response.status.code = autoware_common_msgs::ResponseStatus::SERVICE_UNREADY;
    response.status.message = "Operation mode state is not received";
    return true;
  }
  const bool is_autonomous_driving = is_autonomous_control_active();
  if (is_reroute && is_autonomous_driving && (!reroute_availability_ || !reroute_availability_->availability)) {
    response.status.success = false;
    response.status.code = autoware_common_msgs::ResponseStatus::NO_EFFECT;
    response.status.message = "Cannot reroute as the planner is not in lane following";
    return true;
  }
  PlannerPlugin::RoutePoints points;
  points.push_back(latest_odometry_.pose.pose);
  for (const auto & waypoint : request.waypoints) {
    geometry_msgs::Pose transformed;
    if (!transform_goal_to_map(waypoint, request.header, &transformed)) {
      response.status.success = false;
      response.status.code = autoware_common_msgs::ResponseStatus::TRANSFORM_ERROR;
      response.status.message = "Waypoint cannot be transformed into the map frame";
      return true;
    }
    points.push_back(transformed);
  }
  geometry_msgs::Pose goal_pose;
  if (!transform_goal_to_map(request.goal_pose, request.header, &goal_pose)) {
    response.status.success = false;
    response.status.code = autoware_common_msgs::ResponseStatus::TRANSFORM_ERROR;
    response.status.message = "Goal pose cannot be transformed into the map frame";
    return true;
  }
  points.push_back(goal_pose);
  auto route = planner_->plan(points);
  if (route.segments.empty()) {
    response.status.success = false;
    response.status.message = "DefaultPlanner returned an empty route";
    return true;
  }
  route.header = request.header;
  route.header.frame_id = map_frame_;
  route.uuid = request.uuid;
  route.allow_modification = request.allow_modification;
  if (is_reroute && is_autonomous_driving && !check_reroute_safety(current_route_, route)) {
    response.status.success = false;
    response.status.code = autoware_common_msgs::ResponseStatus::NO_EFFECT;
    response.status.message = "New route is not safe. Reroute failed.";
    return true;
  }
  current_route_ = route;
  original_route_.reset();
  planner_->updateRoute(current_route_);
  autoware_planning_msgs::PoseWithUuidStamped goal;
  goal.header = current_route_.header;
  goal.pose = current_route_.goal_pose;
  goal.uuid = current_route_.uuid;
  arrival_checker_.set_goal(goal);
  publish_route_outputs();
  publish_state(autoware_planning_msgs::RouteState::SET);
  response.status.success = true;
  response.status.message = "Waypoint route set";
  return true;
}
}  // namespace autoware::mission_planner_universe
