#ifndef AUTOWARE_PLANNING_VALIDATOR_REAR_CURRENT_LANE_CORE_HPP_
#define AUTOWARE_PLANNING_VALIDATOR_REAR_CURRENT_LANE_CORE_HPP_

#include <geometry_msgs/Pose.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <memory>

namespace autoware::route_handler
{
class RouteHandler;
}

namespace autoware::planning_validator
{
// Exact route-boundary shape of rear_collision_checker::utils::get_current_lanes.
// A map-only RouteHandler has no valid route lane sequence and deliberately
// yields no lanes rather than falling back to arbitrary map membership.
lanelet::ConstLanelets get_rear_current_lanes(
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const geometry_msgs::Pose & ego_pose, double forward_distance, double backward_distance);

// R003 plugin boundary: consume the node-owned handler without manufacturing
// shared ownership for it.
lanelet::ConstLanelets get_rear_current_lanes(
  const autoware::route_handler::RouteHandler * route_handler,
  const geometry_msgs::Pose & ego_pose, double forward_distance, double backward_distance);
}  // namespace autoware::planning_validator

#endif
