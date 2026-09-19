#include <autoware/behavior_path_goal_planner_module/goal_candidate_search_core.hpp>

#include <autoware/behavior_path_goal_planner_module/goal_candidate_boundary_core.hpp>
#include <autoware/behavior_path_goal_planner_module/goal_candidate_boundary_sampler.hpp>
#include <autoware/trajectory/interpolator/linear.hpp>
#include <autoware/trajectory/pose.hpp>
#include <autoware/trajectory/utils/closest.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <cmath>
#include <utility>

namespace autoware::behavior_path_planner
{
namespace
{
std::optional<autoware::experimental::trajectory::Trajectory<geometry_msgs::Pose>> buildBoundaryTrajectory(
  const lanelet::ConstLineString3d & boundary)
{
  if (boundary.size() < 2U) return std::nullopt;
  std::vector<geometry_msgs::Pose> poses;
  poses.reserve(boundary.size());
  for (auto it = boundary.begin(); it != boundary.end(); ++it) {
    geometry_msgs::Pose pose;
    pose.position.x = it->x(); pose.position.y = it->y(); pose.position.z = it->z();
    const auto next = std::next(it);
    if (next != boundary.end()) {
      geometry_msgs::Point first; first.x = it->x(); first.y = it->y(); first.z = it->z();
      geometry_msgs::Point second; second.x = next->x(); second.y = next->y(); second.z = next->z();
      pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(
        autoware_utils_geometry::calc_azimuth_angle(first, second));
    } else {
      pose.orientation = poses.back().orientation;
    }
    poses.push_back(std::move(pose));
  }
  using autoware::experimental::trajectory::Trajectory;
  using autoware::experimental::trajectory::interpolator::Linear;
  auto trajectory = Trajectory<geometry_msgs::Pose>::Builder()
                      .set_xy_interpolator<Linear>()
                      .set_z_interpolator<Linear>()
                      .build(poses);
  if (!trajectory) return std::nullopt;
  trajectory->align_orientation_with_trajectory_direction();
  return std::move(trajectory.value());
}
}  // namespace

GoalCandidates searchGoalCandidatesAlongBoundary(
  const lanelet::ConstLineString3d & boundary, const geometry_msgs::Pose & reference_goal_pose,
  const GoalCandidateSearchParameters & parameters,
  const autoware_utils_geometry::LinearRing2d & local_vehicle_footprint,
  const GoalCandidateAreaConstraints & area_constraints)
{
  const auto trajectory = buildBoundaryTrajectory(boundary);
  if (!trajectory || parameters.vehicle_width < 0.0 || parameters.vehicle_length < 0.0) return {};
  const auto front_reference = autoware_utils_geometry::calc_offset_pose(
    reference_goal_pose, parameters.base_link2front, 0.0, 0.0);
  const auto front_samples = sampleGoalCandidateBoundary(
    boundary, front_reference, parameters.backward_goal_search_length,
    parameters.forward_goal_search_length, parameters.longitudinal_interval);
  const double reference_s =
    autoware::experimental::trajectory::closest(*trajectory, reference_goal_pose);
  std::vector<GoalCandidateSeed> seeds;
  for (const auto & front_pose : front_samples) {
    const auto rear_pose = autoware_utils_geometry::calc_offset_pose(
      front_pose, -parameters.vehicle_length, 0.0, 0.0);
    const auto minimum_margin_pose = calculateGoalPoseWithMinBoundaryMargin(
      front_pose, rear_pose, boundary, parameters.base_link2front,
      parameters.margin_from_boundary, parameters.vehicle_width / 2.0,
      parameters.left_side_parking);
    if (!minimum_margin_pose) continue;
    const double candidate_s =
      autoware::experimental::trajectory::closest(*trajectory, minimum_margin_pose.value());
    seeds.push_back({minimum_margin_pose.value(), std::abs(candidate_s - reference_s)});
  }
  return generateGoalCandidatesFromSeeds(
    seeds, parameters.left_side_parking, parameters.max_lateral_offset,
    parameters.lateral_offset_interval, local_vehicle_footprint, area_constraints);
}
}  // namespace autoware::behavior_path_planner
