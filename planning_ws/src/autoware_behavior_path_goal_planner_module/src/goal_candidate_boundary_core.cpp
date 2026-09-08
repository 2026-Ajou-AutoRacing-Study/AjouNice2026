#include <autoware/behavior_path_goal_planner_module/goal_candidate_boundary_core.hpp>

#include <autoware/deprecated/boundary_departure_checker/utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>

namespace autoware::behavior_path_planner
{
std::optional<geometry_msgs::Pose> calculateGoalPoseWithMinBoundaryMargin(
  const geometry_msgs::Pose & front_pose, const geometry_msgs::Pose & rear_pose,
  const lanelet::ConstLineString3d & boundary, const double base_link2front,
  const double margin_from_boundary, const double vehicle_half_width,
  const bool left_side_parking)
{
  const auto signed_lateral_distance =
    boundary_departure_checker::utils::calc_signed_lateral_distance_to_boundary(boundary, rear_pose);
  if (!signed_lateral_distance) return std::nullopt;

  const double rear_to_boundary_y = signed_lateral_distance.value();
  const double offset_y = left_side_parking
                            ? rear_to_boundary_y < 0.0
                                ? -(-rear_to_boundary_y + margin_from_boundary + vehicle_half_width)
                                : -(margin_from_boundary + vehicle_half_width)
                            : rear_to_boundary_y < 0.0
                                ? margin_from_boundary + vehicle_half_width
                                : rear_to_boundary_y + margin_from_boundary + vehicle_half_width;
  return autoware_utils_geometry::calc_offset_pose(
    front_pose, -base_link2front, offset_y, 0.0);
}
}  // namespace autoware::behavior_path_planner
