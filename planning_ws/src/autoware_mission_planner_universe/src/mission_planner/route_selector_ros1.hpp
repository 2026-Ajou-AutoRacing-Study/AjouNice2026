#pragma once

#include <ros/ros.h>

#include <autoware_common_msgs/ResponseStatus.h>
#include <autoware_planning_msgs/ClearRoute.h>
#include <autoware_planning_msgs/LaneletRoute.h>
#include <autoware_planning_msgs/RouteState.h>
#include <autoware_planning_msgs/SetLaneletRoute.h>
#include <autoware_planning_msgs/SetWaypointRoute.h>

#include <optional>
#include <variant>

namespace autoware::mission_planner_universe
{
class RouteSelector
{
public:
  RouteSelector(ros::NodeHandle & node, ros::NodeHandle & private_node);

private:
  using ClearRoute = autoware_planning_msgs::ClearRoute;
  using SetWaypointRoute = autoware_planning_msgs::SetWaypointRoute;
  using SetLaneletRoute = autoware_planning_msgs::SetLaneletRoute;
  using LaneletRoute = autoware_planning_msgs::LaneletRoute;
  using RouteState = autoware_planning_msgs::RouteState;
  using ResponseStatus = autoware_common_msgs::ResponseStatus;

  struct RouteInterface {
    RouteState state;
    std::optional<LaneletRoute> route;
    ros::Publisher state_publisher;
    ros::Publisher route_publisher;
  };

  ros::NodeHandle & node_;
  ros::NodeHandle & private_node_;
  RouteInterface main_;
  RouteInterface mrm_;
  ros::ServiceServer main_clear_server_;
  ros::ServiceServer main_waypoint_server_;
  ros::ServiceServer main_lanelet_server_;
  ros::ServiceServer mrm_clear_server_;
  ros::ServiceServer mrm_waypoint_server_;
  ros::ServiceServer mrm_lanelet_server_;
  ros::ServiceClient planner_clear_client_;
  ros::ServiceClient planner_waypoint_client_;
  ros::ServiceClient planner_lanelet_client_;
  ros::Subscriber planner_state_subscriber_;
  ros::Subscriber planner_route_subscriber_;
  bool initialized_{false};
  bool mrm_operating_{false};
  std::variant<std::monostate, SetWaypointRoute::Request, SetLaneletRoute::Request> main_request_;

  static ResponseStatus unavailable(const std::string & service);
  static void stamp(RouteState & state);
  static void ensure_uuid(unique_identifier_msgs::UUID & uuid);
  static void publish_state(RouteInterface & interface, uint8_t state);
  static void publish_route(RouteInterface & interface, const LaneletRoute & route);
  static ResponseStatus call(ros::ServiceClient & client, ClearRoute::Request request);
  static ResponseStatus call(ros::ServiceClient & client, SetWaypointRoute::Request request);
  static ResponseStatus call(ros::ServiceClient & client, SetLaneletRoute::Request request);

  void on_planner_state(const autoware_planning_msgs::RouteStateConstPtr & message);
  void on_planner_route(const autoware_planning_msgs::LaneletRouteConstPtr & message);
  bool on_main_clear(ClearRoute::Request & request, ClearRoute::Response & response);
  bool on_main_waypoint(SetWaypointRoute::Request & request, SetWaypointRoute::Response & response);
  bool on_main_lanelet(SetLaneletRoute::Request & request, SetLaneletRoute::Response & response);
  bool on_mrm_clear(ClearRoute::Request & request, ClearRoute::Response & response);
  bool on_mrm_waypoint(SetWaypointRoute::Request & request, SetWaypointRoute::Response & response);
  bool on_mrm_lanelet(SetLaneletRoute::Request & request, SetLaneletRoute::Response & response);
  ResponseStatus resume_main_route(const ClearRoute::Request & request);
};
}  // namespace autoware::mission_planner_universe
