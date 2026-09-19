#pragma once

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_internal_debug_msgs/Int32Stamped.h>
#include <autoware_map_msgs/LaneletMapBin.h>
#include <autoware_planning_msgs/LaneletRoute.h>
#include <autoware_planning_msgs/SetPreferredPrimitive.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tier4_external_api_msgs/SetPreferredLane.h>
#include <tier4_planning_msgs/RerouteAvailability.h>

#include <optional>
#include <string>
#include <vector>

namespace autoware::manual_lane_change_handler
{
struct LaneChangeRequestResult { std::vector<autoware_planning_msgs::LaneletPrimitive> preferred_primitives; bool success; std::string message; };
enum class Direction { MANUAL_LEFT, MANUAL_RIGHT, AUTO };

class ManualLaneChangeHandler
{
public:
  ManualLaneChangeHandler();
private:
  using Primitive = autoware_planning_msgs::LaneletPrimitive;
  std::vector<Primitive> sort_primitives_left_to_right(const Primitive & preferred, std::vector<Primitive> primitives) const;
  void map_callback(const autoware_map_msgs::LaneletMapBin::ConstPtr & msg);
  void odometry_callback(const nav_msgs::Odometry::ConstPtr & msg) { odometry_ = msg; }
  void route_callback(const autoware_planning_msgs::LaneletRoute::ConstPtr & msg);
  void reroute_callback(const tier4_planning_msgs::RerouteAvailability::ConstPtr & msg) { reroute_availability_ = msg; }
  bool set_preferred_lane(tier4_external_api_msgs::SetPreferredLane::Request & req, tier4_external_api_msgs::SetPreferredLane::Response & res);
  LaneChangeRequestResult process_lane_change_request(int64_t ego_lanelet_id, uint8_t direction);
  void publish_shift_number();

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_map_, sub_odometry_, sub_route_, sub_reroute_availability_;
  ros::ServiceServer srv_set_preferred_lane_;
  ros::ServiceClient client_set_preferred_primitive_;
  ros::Publisher pub_shift_number_;
  autoware::route_handler::RouteHandler route_handler_;
  nav_msgs::Odometry::ConstPtr odometry_;
  tier4_planning_msgs::RerouteAvailability::ConstPtr reroute_availability_;
  autoware_planning_msgs::LaneletRoute::ConstPtr current_route_;
  int8_t shift_number_{0};
};
}  // namespace autoware::manual_lane_change_handler
