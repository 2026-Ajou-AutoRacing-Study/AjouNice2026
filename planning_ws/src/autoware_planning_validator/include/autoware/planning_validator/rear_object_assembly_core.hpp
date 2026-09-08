#ifndef AUTOWARE_PLANNING_VALIDATOR_REAR_OBJECT_ASSEMBLY_CORE_HPP_
#define AUTOWARE_PLANNING_VALIDATOR_REAR_OBJECT_ASSEMBLY_CORE_HPP_

#include "autoware/planning_validator/rear_lanelet_detection_core.hpp"
#include "autoware/planning_validator/rear_object_projection_core.hpp"

#include <memory>
#include <optional>

namespace autoware::route_handler
{
class RouteHandler;
}

namespace autoware::planning_validator
{
// Renders the source RearCollisionChecker::get_pointcloud_object boundary:
// for every lane-bound detection area, retain only points strictly inside its
// polygon, project them onto that area's RouteHandler centerline, and select
// the object with the smallest remaining centerline arc length.  The caller
// must provide already-clustered map-frame points; this function deliberately
// performs no filtering, velocity estimation, or safety decision.
std::optional<RearProjectedObject> assemble_rear_object_from_detection_areas(
  const RearPointCloud & clustered_points, const std::vector<RearDetectionArea> & detection_areas,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler);

// R003 consumes the already node-owned handler without manufacturing shared
// ownership across the provider boundary.
std::optional<RearProjectedObject> assemble_rear_object_from_detection_areas(
  const RearPointCloud & clustered_points, const std::vector<RearDetectionArea> & detection_areas,
  const autoware::route_handler::RouteHandler * route_handler);
}  // namespace autoware::planning_validator

#endif
