#ifndef AUTOWARE_PLANNING_VALIDATOR_REAR_OBJECT_PROJECTION_CORE_HPP_
#define AUTOWARE_PLANNING_VALIDATOR_REAR_OBJECT_PROJECTION_CORE_HPP_

#include "autoware/planning_validator/rear_pointcloud_core.hpp"

#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <geometry_msgs/Pose.h>

#include <cstdint>
#include <optional>

namespace autoware::planning_validator
{
struct RearProjectedObject
{
  geometry_msgs::Pose pose;
  double absolute_distance{0.0};
  // Lane at the far end of the selected detection area.  Downstream tracking
  // needs this identity to look up predecessors exactly as the source does.
  int64_t furthest_lane_id{0};
};

// Mirrors the source object's center-line projection: resample the supplied
// RouteHandler centerline at 2 m and keep the projected obstacle with the
// smallest remaining arc length to the end of that lane sequence.
std::optional<RearProjectedObject> project_rear_object_to_centerline(
  const RearPointCloud & clustered_points,
  const autoware_internal_planning_msgs::PathWithLaneId & centerline_path);
}  // namespace autoware::planning_validator

#endif
