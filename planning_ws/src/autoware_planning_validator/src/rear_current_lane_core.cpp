#include "autoware/planning_validator/rear_current_lane_core.hpp"

#include <autoware/route_handler/route_handler.hpp>

namespace autoware::planning_validator
{
lanelet::ConstLanelets get_rear_current_lanes(
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const geometry_msgs::Pose & ego_pose, const double forward_distance, const double backward_distance)
{
  return get_rear_current_lanes(
    route_handler.get(), ego_pose, forward_distance, backward_distance);
}

lanelet::ConstLanelets get_rear_current_lanes(
  const autoware::route_handler::RouteHandler * route_handler,
  const geometry_msgs::Pose & ego_pose, const double forward_distance, const double backward_distance)
{
  if (!route_handler) return {};
  lanelet::ConstLanelet closest_lanelet;
  if (!route_handler->getClosestLaneletWithinRoute(ego_pose, &closest_lanelet)) return {};
  return route_handler->getLaneletSequence(
    closest_lanelet, ego_pose, backward_distance, forward_distance);
}
}  // namespace autoware::planning_validator
