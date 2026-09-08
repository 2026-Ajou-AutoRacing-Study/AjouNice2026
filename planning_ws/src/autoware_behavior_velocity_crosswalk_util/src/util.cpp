#include <autoware/behavior_velocity_crosswalk_module/util.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/trajectory/utils/crossed.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/road_marking.hpp>

namespace autoware::behavior_velocity_planner
{
std::vector<std::pair<int64_t, lanelet::ConstLanelet>> getCrosswalksOnPath(
  const geometry_msgs::Pose & current_pose, const autoware_internal_planning_msgs::PathWithLaneId & path,
  const lanelet::LaneletMapPtr lanelet_map,
  const std::shared_ptr<const lanelet::routing::RoutingGraphContainer> & overall_graphs)
{
  std::vector<std::pair<int64_t, lanelet::ConstLanelet>> result;
  if (!lanelet_map || !overall_graphs) return result;
  const auto nearest = planning_utils::getNearestLaneId(path, lanelet_map, current_pose);
  const auto ids = nearest ? planning_utils::getSubsequentLaneIdsSetOnPath(path, *nearest) :
    planning_utils::getSortedLaneIdsFromPath(path);
  for (const auto id : ids) {
    if (!lanelet_map->laneletLayer.exists(id)) continue;
    constexpr int pedestrian_graph_id = 1;
    for (const auto & crosswalk : overall_graphs->conflictingInGraph(
           lanelet_map->laneletLayer.get(id), pedestrian_graph_id)) result.emplace_back(id, crosswalk);
  }
  return result;
}
std::set<int64_t> getCrosswalkIdSetOnPath(const geometry_msgs::Pose & current_pose,
  const autoware_internal_planning_msgs::PathWithLaneId & path, const lanelet::LaneletMapPtr lanelet_map,
  const std::shared_ptr<const lanelet::routing::RoutingGraphContainer> & overall_graphs)
{
  std::set<int64_t> result;
  for (const auto & pair : getCrosswalksOnPath(current_pose, path, lanelet_map, overall_graphs)) result.insert(pair.second.id());
  return result;
}
std::optional<std::pair<geometry_msgs::Point, geometry_msgs::Point>> getPathEndPointsOnCrosswalk(
  const experimental::Trajectory & ego_path, const lanelet::BasicPolygon2d & polygon,
  const geometry_msgs::Point & ego_pos)
{
  const auto crossings = autoware::experimental::trajectory::crossed_with_polygon(ego_path, polygon);
  if (crossings.empty()) return std::nullopt;
  const auto first = ego_path.compute(crossings.front()).point.pose.position;
  const auto last = ego_path.compute(crossings.back()).point.pose.position;
  geometry_msgs::Point first_msg, last_msg;
  first_msg.x = first.x; first_msg.y = first.y; first_msg.z = ego_pos.z;
  last_msg.x = last.x; last_msg.y = last.y; last_msg.z = ego_pos.z;
  return std::make_pair(first_msg, last_msg);
}
std::optional<lanelet::ConstLineString3d> getStopLineFromMap(lanelet::Id lane_id,
  const lanelet::LaneletMapPtr & lanelet_map_ptr, const std::string & attribute_name)
{
  if (!lanelet_map_ptr || !lanelet_map_ptr->laneletLayer.exists(lane_id)) return std::nullopt;
  const auto lane = lanelet_map_ptr->laneletLayer.get(lane_id);
  for (const auto & marking : lane.regulatoryElementsAs<lanelet::autoware::RoadMarking>()) {
    const auto line = marking->roadMarking();
    const auto type = line.attributeOr(lanelet::AttributeName::Type, "none");
    const auto target = line.attributeOr(attribute_name, 0);
    if (type == lanelet::AttributeValueString::StopLine && target == lane_id) return line;
  }
  return std::nullopt;
}
}  // namespace autoware::behavior_velocity_planner
