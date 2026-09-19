#include <autoware/behavior_path_goal_planner_module/goal_candidate_boundary_sampler.hpp>

#include <autoware/trajectory/interpolator/linear.hpp>
#include <autoware/trajectory/pose.hpp>
#include <autoware/trajectory/utils/closest.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <algorithm>
#include <utility>

namespace autoware::behavior_path_planner
{
std::vector<geometry_msgs::Pose> sampleGoalCandidateBoundary(
  const lanelet::ConstLineString3d & boundary, const geometry_msgs::Pose & reference_pose,
  const double backward_length, const double forward_length, const double interval)
{
  if (boundary.size() < 2U || interval <= 0.0) return {};
  std::vector<geometry_msgs::Pose> boundary_poses;
  boundary_poses.reserve(boundary.size());
  for (auto it = boundary.begin(); it != boundary.end(); ++it) {
    geometry_msgs::Pose pose;
    pose.position.x = it->x();
    pose.position.y = it->y();
    pose.position.z = it->z();
    const auto next = std::next(it);
    if (next != boundary.end()) {
      geometry_msgs::Point current;
      current.x = it->x(); current.y = it->y(); current.z = it->z();
      geometry_msgs::Point following;
      following.x = next->x(); following.y = next->y(); following.z = next->z();
      pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(
        autoware_utils_geometry::calc_azimuth_angle(current, following));
    } else {
      pose.orientation = boundary_poses.back().orientation;
    }
    boundary_poses.push_back(std::move(pose));
  }

  using autoware::experimental::trajectory::Trajectory;
  using autoware::experimental::trajectory::interpolator::Linear;
  auto trajectory = Trajectory<geometry_msgs::Pose>::Builder()
                      .set_xy_interpolator<Linear>()
                      .set_z_interpolator<Linear>()
                      .build(boundary_poses);
  if (!trajectory) return {};
  trajectory->align_orientation_with_trajectory_direction();
  const double reference_s = autoware::experimental::trajectory::closest(*trajectory, reference_pose);
  const double start_s = std::max(0.0, reference_s - backward_length);
  const double end_s = std::min(trajectory->length(), reference_s + forward_length);
  if (end_s <= start_s) return {};

  std::vector<geometry_msgs::Pose> samples;
  for (double s = start_s; s < end_s; s += interval) {
    auto pose = trajectory->compute(s);
    pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(trajectory->azimuth(s));
    samples.push_back(pose);
  }
  return samples;
}
}  // namespace autoware::behavior_path_planner
