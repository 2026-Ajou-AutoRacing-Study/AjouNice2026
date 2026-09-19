#include "route_selector_ros1.hpp"

#include <autoware/qos_utils/qos_compatibility.hpp>

#include <algorithm>
#include <random>

namespace autoware::mission_planner_universe
{
RouteSelector::RouteSelector(ros::NodeHandle & node, ros::NodeHandle & private_node)
: node_(node), private_node_(private_node)
{
  main_.state_publisher = node_.advertise<RouteState>("main/state", 1, true);
  main_.route_publisher = node_.advertise<LaneletRoute>("main/route", 1, true);
  mrm_.state_publisher = node_.advertise<RouteState>("mrm/state", 1, true);
  mrm_.route_publisher = node_.advertise<LaneletRoute>("mrm/route", 1, true);
  main_clear_server_ = node_.advertiseService("main/clear_route", &RouteSelector::on_main_clear, this);
  main_waypoint_server_ = node_.advertiseService("main/set_waypoint_route", &RouteSelector::on_main_waypoint, this);
  main_lanelet_server_ = node_.advertiseService("main/set_lanelet_route", &RouteSelector::on_main_lanelet, this);
  mrm_clear_server_ = node_.advertiseService("mrm/clear_route", &RouteSelector::on_mrm_clear, this);
  mrm_waypoint_server_ = node_.advertiseService("mrm/set_waypoint_route", &RouteSelector::on_mrm_waypoint, this);
  mrm_lanelet_server_ = node_.advertiseService("mrm/set_lanelet_route", &RouteSelector::on_mrm_lanelet, this);
  planner_clear_client_ = autoware::qos_utils::ordered_service_clients().create<ClearRoute>(
    "planner/clear_route");
  planner_waypoint_client_ = autoware::qos_utils::ordered_service_clients().create<SetWaypointRoute>(
    "planner/set_waypoint_route");
  planner_lanelet_client_ = autoware::qos_utils::ordered_service_clients().create<SetLaneletRoute>(
    "planner/set_lanelet_route");
  planner_state_subscriber_ = node_.subscribe("planner/state", 1, &RouteSelector::on_planner_state, this);
  planner_route_subscriber_ = node_.subscribe("planner/route", 1, &RouteSelector::on_planner_route, this);
  publish_state(main_, RouteState::INITIALIZING);
  publish_state(mrm_, RouteState::INITIALIZING);
}

RouteSelector::ResponseStatus RouteSelector::unavailable(const std::string & service)
{
  ResponseStatus status;
  status.success = false;
  status.code = ResponseStatus::SERVICE_UNREADY;
  status.message = service;
  return status;
}

void RouteSelector::stamp(RouteState & state)
{
  const auto now = ros::Time::now();
  state.stamp.sec = now.sec;
  state.stamp.nanosec = now.nsec;
}

void RouteSelector::ensure_uuid(unique_identifier_msgs::UUID & uuid)
{
  const bool empty = std::all_of(uuid.uuid.begin(), uuid.uuid.end(), [](uint8_t value) { return value == 0; });
  if (!empty) return;
  static std::independent_bits_engine<std::mt19937, 8, uint8_t> generator(std::random_device{}());
  std::generate(uuid.uuid.begin(), uuid.uuid.end(), std::ref(generator));
}

void RouteSelector::publish_state(RouteInterface & interface, uint8_t state)
{
  stamp(interface.state);
  interface.state.state = state;
  interface.state_publisher.publish(interface.state);
}

void RouteSelector::publish_route(RouteInterface & interface, const LaneletRoute & route)
{
  interface.route = route;
  interface.route_publisher.publish(route);
}

RouteSelector::ResponseStatus RouteSelector::call(ros::ServiceClient & client, ClearRoute::Request request)
{
  ClearRoute service; service.request = request;
  return client.exists() && client.call(service) ? service.response.status : unavailable(client.getService());
}
RouteSelector::ResponseStatus RouteSelector::call(ros::ServiceClient & client, SetWaypointRoute::Request request)
{
  SetWaypointRoute service; service.request = request;
  return client.exists() && client.call(service) ? service.response.status : unavailable(client.getService());
}
RouteSelector::ResponseStatus RouteSelector::call(ros::ServiceClient & client, SetLaneletRoute::Request request)
{
  SetLaneletRoute service; service.request = request;
  return client.exists() && client.call(service) ? service.response.status : unavailable(client.getService());
}

void RouteSelector::on_planner_state(const autoware_planning_msgs::RouteStateConstPtr & message)
{
  if (message->state == RouteState::UNSET && !initialized_) { initialized_ = true; publish_state(main_, RouteState::UNSET); publish_state(mrm_, RouteState::UNSET); }
  auto & interface = mrm_operating_ ? mrm_ : main_;
  interface.state = *message;
  interface.state_publisher.publish(interface.state);
}
void RouteSelector::on_planner_route(const autoware_planning_msgs::LaneletRouteConstPtr & message)
{
  publish_route(mrm_operating_ ? mrm_ : main_, *message);
}

bool RouteSelector::on_main_clear(ClearRoute::Request & request, ClearRoute::Response & response)
{
  main_request_ = std::monostate{}; main_.route.reset();
  response.status = mrm_operating_ ? (publish_state(main_, RouteState::UNSET), ResponseStatus{}) : call(planner_clear_client_, request);
  if (mrm_operating_) response.status.success = true;
  return true;
}
bool RouteSelector::on_main_waypoint(SetWaypointRoute::Request & request, SetWaypointRoute::Response & response)
{
  ensure_uuid(request.uuid); main_request_ = request; main_.route.reset();
  if (mrm_operating_) { publish_state(main_, RouteState::INTERRUPTED); response.status.success = true; return true; }
  response.status = call(planner_waypoint_client_, request); return true;
}
bool RouteSelector::on_main_lanelet(SetLaneletRoute::Request & request, SetLaneletRoute::Response & response)
{
  ensure_uuid(request.uuid); main_request_ = request; main_.route.reset();
  if (mrm_operating_) { publish_state(main_, RouteState::INTERRUPTED); response.status.success = true; return true; }
  response.status = call(planner_lanelet_client_, request); return true;
}
bool RouteSelector::on_mrm_waypoint(SetWaypointRoute::Request & request, SetWaypointRoute::Response & response)
{
  ensure_uuid(request.uuid); response.status = call(planner_waypoint_client_, request);
  if (response.status.success) { mrm_operating_ = true; if (main_.state.state != RouteState::UNSET) publish_state(main_, RouteState::INTERRUPTED); }
  return true;
}
bool RouteSelector::on_mrm_lanelet(SetLaneletRoute::Request & request, SetLaneletRoute::Response & response)
{
  ensure_uuid(request.uuid); response.status = call(planner_lanelet_client_, request);
  if (response.status.success) { mrm_operating_ = true; if (main_.state.state != RouteState::UNSET) publish_state(main_, RouteState::INTERRUPTED); }
  return true;
}
bool RouteSelector::on_mrm_clear(ClearRoute::Request & request, ClearRoute::Response & response)
{
  response.status = resume_main_route(request);
  if (response.status.success) { mrm_operating_ = false; publish_state(mrm_, RouteState::UNSET); }
  return true;
}

RouteSelector::ResponseStatus RouteSelector::resume_main_route(const ClearRoute::Request & request)
{
  if (std::holds_alternative<std::monostate>(main_request_)) return call(planner_clear_client_, request);
  if (main_.route) {
    SetLaneletRoute::Request restore;
    restore.header = main_.route->header; restore.goal_pose = main_.route->goal_pose; restore.segments = main_.route->segments; restore.uuid = main_.route->uuid; restore.allow_modification = main_.route->allow_modification;
    const auto status = call(planner_lanelet_client_, restore); if (status.success) return status;
  }
  if (const auto * waypoint = std::get_if<SetWaypointRoute::Request>(&main_request_)) return call(planner_waypoint_client_, *waypoint);
  if (const auto * lanelet = std::get_if<SetLaneletRoute::Request>(&main_request_)) {
    SetWaypointRoute::Request restore; restore.header = lanelet->header; restore.goal_pose = lanelet->goal_pose; restore.uuid = lanelet->uuid; restore.allow_modification = lanelet->allow_modification;
    return call(planner_waypoint_client_, restore);
  }
  return unavailable("planner route request");
}
}  // namespace autoware::mission_planner_universe
