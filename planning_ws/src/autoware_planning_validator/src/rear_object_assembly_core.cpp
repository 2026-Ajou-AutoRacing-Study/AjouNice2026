#include "autoware/planning_validator/rear_object_assembly_core.hpp"

#include "autoware/planning_validator/rear_polygon_selection_core.hpp"

#include <autoware/route_handler/route_handler.hpp>

#include <limits>

namespace autoware::planning_validator
{
std::optional<RearProjectedObject> assemble_rear_object_from_detection_areas(
  const RearPointCloud & clustered_points, const std::vector<RearDetectionArea> & detection_areas,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler)
{
  return assemble_rear_object_from_detection_areas(
    clustered_points, detection_areas, route_handler.get());
}

std::optional<RearProjectedObject> assemble_rear_object_from_detection_areas(
  const RearPointCloud & clustered_points, const std::vector<RearDetectionArea> & detection_areas,
  const autoware::route_handler::RouteHandler * route_handler)
{
  if (clustered_points.empty() || !route_handler) return std::nullopt;

  std::optional<RearProjectedObject> result;
  for (const auto & area : detection_areas) {
    if (area.polygon.size() < 3U || area.lanes.empty()) continue;
    RearDetectionPolygon polygon;
    polygon.reserve(area.polygon.size());
    for (const auto & point : area.polygon) {
      polygon.push_back({static_cast<float>(point.x()), static_cast<float>(point.y()),
        static_cast<float>(point.z())});
    }
    const auto selected = select_rear_points_in_detection_polygons({polygon}, clustered_points);
    if (selected->empty()) continue;

    const auto centerline = route_handler->getCenterLinePath(
      area.lanes, 0.0, std::numeric_limits<double>::max());
    const auto projected = project_rear_object_to_centerline(*selected, centerline);
    if (!projected.has_value()) continue;
    if (!result.has_value() || projected->absolute_distance < result->absolute_distance) {
      result = projected;
      result->furthest_lane_id = area.lanes.back().id();
    }
  }
  return result;
}
}  // namespace autoware::planning_validator
