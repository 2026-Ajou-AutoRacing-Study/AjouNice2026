#include "route_selector_ros1.hpp"

#include <ros/ros.h>

#include <autoware_planning_msgs/ClearRoute.h>
#include <autoware_planning_msgs/SetLaneletRoute.h>
#include <autoware_planning_msgs/SetWaypointRoute.h>

#include <algorithm>
#include <atomic>
#include <iostream>

namespace
{
using ClearRoute = autoware_planning_msgs::ClearRoute;
using SetLaneletRoute = autoware_planning_msgs::SetLaneletRoute;
using SetWaypointRoute = autoware_planning_msgs::SetWaypointRoute;
void accept(autoware_common_msgs::ResponseStatus & status) { status.success = true; status.code = 0; }
bool has_uuid(const unique_identifier_msgs::UUID & uuid)
{
  return std::any_of(uuid.uuid.begin(), uuid.uuid.end(), [](const uint8_t value) { return value != 0; });
}
}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "route_selector_service_proxy_oracle");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");
  std::atomic<int> clear_calls{0}, waypoint_calls{0}, lanelet_calls{0};
  std::atomic<bool> waypoint_uuid{false}, lanelet_uuid{false};
  auto clear_server = node.advertiseService<ClearRoute::Request, ClearRoute::Response>(
    "planner/clear_route", [&](ClearRoute::Request &, ClearRoute::Response & response) { ++clear_calls; accept(response.status); return true; });
  auto waypoint_server = node.advertiseService<SetWaypointRoute::Request, SetWaypointRoute::Response>(
    "planner/set_waypoint_route", [&](SetWaypointRoute::Request & request, SetWaypointRoute::Response & response) { ++waypoint_calls; waypoint_uuid = has_uuid(request.uuid); accept(response.status); return true; });
  auto lanelet_server = node.advertiseService<SetLaneletRoute::Request, SetLaneletRoute::Response>(
    "planner/set_lanelet_route", [&](SetLaneletRoute::Request & request, SetLaneletRoute::Response & response) { ++lanelet_calls; lanelet_uuid = has_uuid(request.uuid); accept(response.status); return true; });
  autoware::mission_planner_universe::RouteSelector selector(node, private_node);
  ros::AsyncSpinner spinner(2); spinner.start();
  auto clear_client = node.serviceClient<ClearRoute>("main/clear_route");
  auto waypoint_client = node.serviceClient<SetWaypointRoute>("main/set_waypoint_route");
  auto lanelet_client = node.serviceClient<SetLaneletRoute>("main/set_lanelet_route");
  if (!clear_client.waitForExistence(ros::Duration(1.0)) || !waypoint_client.waitForExistence(ros::Duration(1.0)) || !lanelet_client.waitForExistence(ros::Duration(1.0))) return 2;
  ClearRoute clear; SetWaypointRoute waypoint; waypoint.request.goal_pose.orientation.w = 1.0;
  SetLaneletRoute lanelet; lanelet.request.goal_pose.orientation.w = 1.0;
  if (!clear_client.call(clear) || !clear.response.status.success || !waypoint_client.call(waypoint) || !waypoint.response.status.success || !lanelet_client.call(lanelet) || !lanelet.response.status.success) return 3;
  if (clear_calls != 1 || waypoint_calls != 1 || lanelet_calls != 1 || !waypoint_uuid || !lanelet_uuid) return 4;
  std::cout << "route_selector_service_proxy_oracle=PASS clear=" << clear_calls << " waypoint=" << waypoint_calls << " lanelet=" << lanelet_calls << std::endl;
  return 0;
}
