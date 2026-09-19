#include "autoware/planning_validator_intersection_collision_checker/ego_trajectory_core_ros1.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/geometry.hpp>

namespace autoware::planning_validator::intersection_collision_checker
{
std::optional<EgoTrajectory> make_ego_trajectory(
  const autoware_planning_msgs::Trajectory & resampled_trajectory,
  const geometry_msgs::Pose & ego_pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  if (resampled_trajectory.points.empty()) return std::nullopt;
  constexpr float minimum_trajectory_velocity = 0.1F;
  const auto front_pose = autoware_utils_geometry::calc_offset_pose(
    ego_pose, vehicle_info.front_overhang_m + vehicle_info.wheel_base_m, 0.0, 0.0);
  const auto back_pose = autoware_utils_geometry::calc_offset_pose(
    ego_pose, -vehicle_info.rear_overhang_m, 0.0, 0.0);
  EgoTrajectory result;
  result.front_trajectory = resampled_trajectory.points;
  result.back_trajectory = resampled_trajectory.points;
  autoware::motion_utils::calculate_time_from_start(
    result.front_trajectory, front_pose.position, minimum_trajectory_velocity);
  autoware::motion_utils::calculate_time_from_start(
    result.back_trajectory, back_pose.position, minimum_trajectory_velocity);
  result.front_index =
    autoware::motion_utils::findNearestIndex(result.front_trajectory, front_pose.position);
  result.back_index =
    autoware::motion_utils::findNearestIndex(result.back_trajectory, back_pose.position);
  return result;
}
}  // namespace autoware::planning_validator::intersection_collision_checker
