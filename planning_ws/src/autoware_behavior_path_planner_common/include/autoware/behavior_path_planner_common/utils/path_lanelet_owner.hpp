#ifndef AUTOWARE_BPP_PATH_LANELET_OWNER_HPP
#define AUTOWARE_BPP_PATH_LANELET_OWNER_HPP
#include <autoware/route_handler/route_lanelet_tie.hpp>
#include <geometry_msgs/Pose.h>
#include <tf2/utils.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <boost/geometry/algorithms/comparable_distance.hpp>
#include <optional>

namespace autoware::behavior_path_planner::utils
{
// Only path-eligible IDs are supplied. Polygon distance remains primary;
// exact ties use the same geometric ordering as the route-root selector.
inline std::optional<lanelet::ConstLanelet> getPathLaneletOwner(
  const lanelet::ConstLanelets & lanes, const geometry_msgs::Pose & pose)
{
  if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y)) return {};
  const double yaw = tf2::getYaw(pose.orientation);
  if (!std::isfinite(yaw)) return {};
  std::optional<lanelet::ConstLanelet> result;
  double minimum = std::numeric_limits<double>::infinity();
  autoware::route_handler::detail::RouteLaneletTie best;
  for (const auto & lane : lanes) {
    const double distance = boost::geometry::comparable_distance(
      lanelet::BasicPoint2d(pose.position.x, pose.position.y), lane.polygon2d().basicPolygon());
    if (!std::isfinite(distance)) continue;
    const auto score = autoware::route_handler::detail::routeLaneletTie(
      lane.centerline(), pose.position.x, pose.position.y, yaw, lane.id());
    if (distance < minimum || (distance == minimum &&
        (!result || autoware::route_handler::detail::routeLaneletTieLess(score, best)))) {
      result = lane; minimum = distance; best = score;
    }
  }
  return result;
}
}  // namespace autoware::behavior_path_planner::utils
#endif
