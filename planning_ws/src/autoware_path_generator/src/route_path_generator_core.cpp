#include <autoware/path_generator/route_path_generator_core.hpp>
#include <autoware/trajectory/path_point_with_lane_id.hpp>

namespace autoware::path_generator
{
std::optional<autoware_internal_planning_msgs::PathWithLaneId> generateRoutePath(
  const autoware_map_msgs::LaneletMapBin & map, const autoware_planning_msgs::LaneletRoute & route,
  const geometry_msgs::Pose & initial_pose, const geometry_msgs::Pose & current_pose,
  const RoutePathGeneratorParameters & parameters)
{
  if (route.segments.empty() || parameters.forward_length < 0.0 || parameters.backward_length < 0.0) {
    return std::nullopt;
  }
  auto manager = experimental::lanelet2_utils::RouteManager::create(map, route, initial_pose);
  if (!manager) return std::nullopt;
  manager = std::move(*manager).update_current_pose(
    current_pose, parameters.ego_nearest_dist_threshold, parameters.ego_nearest_yaw_threshold);
  if (!manager) return std::nullopt;
  const auto sequence = manager->get_lanelet_sequence_outward_route(
    parameters.forward_length, parameters.backward_length).as_lanelets();
  if (sequence.empty()) return std::nullopt;
  const auto path = experimental::trajectory::build_reference_path(
    sequence, manager->current_lanelet(), current_pose, manager->lanelet_map_ptr(),
    manager->routing_graph_ptr(), manager->traffic_rules_ptr(), parameters.forward_length,
    parameters.backward_length);
  if (!path) return std::nullopt;
  autoware_internal_planning_msgs::PathWithLaneId output;
  output.header = route.header;
  output.points = path->restore();
  if (output.points.empty()) return std::nullopt;
  return output;
}
}  // namespace autoware::path_generator
