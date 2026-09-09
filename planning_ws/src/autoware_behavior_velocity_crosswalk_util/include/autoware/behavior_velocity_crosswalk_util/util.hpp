#pragma once

#include <autoware/behavior_velocity_planner_common/experimental/plugin_interface.hpp>
#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <lanelet2_core/primitives/Polygon.h>
#include <lanelet2_routing/RoutingGraphContainer.h>

#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{
std::vector<std::pair<int64_t, lanelet::ConstLanelet>> getCrosswalksOnPath(
  const geometry_msgs::Pose & current_pose, const autoware_internal_planning_msgs::PathWithLaneId & path,
  const lanelet::LaneletMapPtr lanelet_map,
  const std::shared_ptr<const lanelet::routing::RoutingGraphContainer> & overall_graphs);
std::set<int64_t> getCrosswalkIdSetOnPath(const geometry_msgs::Pose & current_pose,
  const autoware_internal_planning_msgs::PathWithLaneId & path, const lanelet::LaneletMapPtr lanelet_map,
  const std::shared_ptr<const lanelet::routing::RoutingGraphContainer> & overall_graphs);
std::optional<std::pair<geometry_msgs::Point, geometry_msgs::Point>> getPathEndPointsOnCrosswalk(
  const experimental::Trajectory & ego_path, const lanelet::BasicPolygon2d & polygon,
  const geometry_msgs::Point & ego_pos);
std::optional<lanelet::ConstLineString3d> getStopLineFromMap(lanelet::Id lane_id,
  const lanelet::LaneletMapPtr lanelet_map_ptr, const std::string & attribute_name);
}  // namespace autoware::behavior_velocity_planner
