#include "autoware/planning_validator/rear_object_projection_core.hpp"

#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <geometry_msgs/Point.h>

#include <limits>

namespace autoware::planning_validator
{
std::optional<RearProjectedObject> project_rear_object_to_centerline(
  const RearPointCloud & clustered_points,
  const autoware_internal_planning_msgs::PathWithLaneId & centerline_path)
{
  if (clustered_points.empty() || centerline_path.points.size() < 2U) return std::nullopt;
  const auto resampled_path = autoware::motion_utils::resamplePath(centerline_path, 2.0);
  if (resampled_path.points.size() < 2U) return std::nullopt;

  std::optional<RearProjectedObject> result;
  for (const auto & point : clustered_points) {
    geometry_msgs::Point point_geom;
    point_geom.x = point.x;
    point_geom.y = point.y;
    point_geom.z = point.z;
    const size_t segment_index =
      autoware::motion_utils::findNearestSegmentIndex(resampled_path.points, point_geom);
    const double signed_offset = autoware::motion_utils::calcLongitudinalOffsetToSegment(
      resampled_path.points, segment_index, point_geom);
    const double remaining_arc_length =
      autoware::motion_utils::calcSignedArcLength(
        resampled_path.points, segment_index, resampled_path.points.size() - 1U) - signed_offset;
    const auto pose = autoware::motion_utils::calcLongitudinalOffsetPose(
      resampled_path.points, segment_index, signed_offset);
    if (!pose.has_value()) continue;
    if (!result.has_value() || result->absolute_distance > remaining_arc_length) {
      result = RearProjectedObject{pose.value(), remaining_arc_length};
    }
  }
  return result;
}
}  // namespace autoware::planning_validator
