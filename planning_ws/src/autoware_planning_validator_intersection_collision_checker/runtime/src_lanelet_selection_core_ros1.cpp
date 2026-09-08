#include "autoware/planning_validator_intersection_collision_checker/lanelet_selection_core_ros1.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/route_handler/route_handler.hpp>

#include <algorithm>

namespace autoware::planning_validator::intersection_collision_checker
{
namespace
{
bool is_turn_lanelet(const lanelet::ConstLanelet & lanelet)
{
  return lanelet.hasAttribute("turn_direction") && lanelet.attribute("turn_direction") != "straight";
}
}  // namespace

std::optional<LaneletSelection> select_trajectory_lanelets(
  const autoware_planning_msgs::Trajectory & trajectory,
  const autoware::route_handler::RouteHandler & route_handler,
  const geometry_msgs::Pose & ego_pose)
{
  // The source throws after the route-bound closest query fails.  At the
  // plugin boundary this is represented as no selection, which fails closed.
  if (!route_handler.isHandlerReady() || trajectory.points.empty()) return std::nullopt;

  lanelet::ConstLanelet closest_lanelet;
  if (!route_handler.getClosestLaneletWithinRoute(ego_pose, &closest_lanelet)) return std::nullopt;

  LaneletSelection selected;
  const auto forward_length = autoware::motion_utils::calcSignedArcLength(
    trajectory.points, ego_pose.position, trajectory.points.size() - 1U);
  selected.trajectory_lanelets =
    route_handler.getLaneletSequence(closest_lanelet, ego_pose, 0.0, forward_length);
  if (selected.trajectory_lanelets.empty()) return std::nullopt;

  lanelet::ConstLanelets previous{closest_lanelet};
  if (is_turn_lanelet(closest_lanelet)) {
    while (route_handler.getPreviousLaneletsWithinRoute(previous.front(), &previous) &&
           is_turn_lanelet(previous.front())) {
      selected.trajectory_lanelets.insert(selected.trajectory_lanelets.begin(), previous.front());
    }
  }

  if (route_handler.getPreviousLaneletsWithinRoute(closest_lanelet, &previous)) {
    selected.connected_lanelets.push_back(previous.front());
    for (const auto & connected : route_handler.getNextLanelets(previous.front())) {
      selected.connected_lanelets.push_back(connected);
    }
  }

  bool lock_turn_lanelets = false;
  for (const auto & lanelet : selected.trajectory_lanelets) {
    if (!lock_turn_lanelets) {
      if (is_turn_lanelet(lanelet)) {
        selected.turn_lanelets.push_back(lanelet);
      } else if (!selected.turn_lanelets.empty()) {
        lock_turn_lanelets = true;
      }
    }
    for (const auto & connected : route_handler.getNextLanelets(lanelet)) {
      selected.connected_lanelets.push_back(connected);
    }
  }
  return selected;
}
}  // namespace autoware::planning_validator::intersection_collision_checker
