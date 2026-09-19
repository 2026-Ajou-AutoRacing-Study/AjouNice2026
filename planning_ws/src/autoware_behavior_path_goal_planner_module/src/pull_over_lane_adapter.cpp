#include <autoware/behavior_path_goal_planner_module/pull_over_lane_adapter.hpp>

namespace autoware::behavior_path_planner {
std::optional<lanelet::ConstLanelets> selectPullOverLanes(
  const std::shared_ptr<const autoware::route_handler::RouteHandler> & route_handler,
  const geometry_msgs::Pose & goal_pose, const bool left_side, const double backward_distance,
  const double forward_distance)
{
  if (!route_handler || !route_handler->isHandlerReady()) return std::nullopt;
  const auto backward_with_buffer = backward_distance + 100.0;
  if (const auto target = route_handler->getPullOverTarget(goal_pose)) {
    const auto lanes = route_handler->getShoulderLaneletSequence(
      *target, goal_pose, backward_with_buffer, forward_distance);
    return lanes.empty() ? std::nullopt : std::make_optional(lanes);
  }
  lanelet::ConstLanelet closest;
  if (!route_handler->getClosestLaneletWithinRoute(goal_pose, &closest)) return std::nullopt;
  const auto outermost = left_side
    ? route_handler->getMostLeftLanelet(closest, false, true)
    : route_handler->getMostRightLanelet(closest, false, true);
  if (outermost.id() == lanelet::InvalId) return std::nullopt;
  if (route_handler->isShoulderLanelet(outermost)) {
    const auto lanes = route_handler->get_shoulder_lanelet_sequence(
      outermost, backward_with_buffer, forward_distance);
    return lanes.empty() ? std::nullopt : std::make_optional(lanes);
  }
  return lanelet::ConstLanelets{outermost};
}
}
