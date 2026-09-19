#include <autoware/motion_velocity_planner_common/utils.hpp>

#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <algorithm>

namespace autoware::motion_velocity_planner::utils
{
namespace
{
autoware_planning_msgs::TrajectoryPoint extendTrajectoryPoint(
  const double extend_distance, const autoware_planning_msgs::TrajectoryPoint & goal_point,
  const bool is_driving_forward)
{
  autoware_planning_msgs::TrajectoryPoint extended;
  extended.pose = autoware_utils_geometry::calc_offset_pose(
    goal_point.pose, extend_distance * (is_driving_forward ? 1.0 : -1.0), 0.0, 0.0);
  extended.longitudinal_velocity_mps = goal_point.longitudinal_velocity_mps;
  extended.lateral_velocity_mps = goal_point.lateral_velocity_mps;
  extended.acceleration_mps2 = goal_point.acceleration_mps2;
  return extended;
}

std::vector<autoware_planning_msgs::TrajectoryPoint> resampleTrajectoryPoints(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory_points,
  const double interval)
{
  const auto trajectory = autoware::motion_utils::convertToTrajectory(trajectory_points);
  const auto resampled = autoware::motion_utils::resampleTrajectory(trajectory, interval);
  auto points = autoware::motion_utils::convertToTrajectoryPointArray(resampled);
  const auto driving_forward = autoware::motion_utils::isDrivingForward(trajectory_points);
  autoware::motion_utils::insertOrientationAsSpline(
    points, driving_forward.value_or(true));
  return points;
}
}  // namespace

std::vector<autoware_planning_msgs::TrajectoryPoint> get_extended_trajectory_points(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & input_points,
  const double extend_distance, const double step_length)
{
  auto output = input_points;
  if (input_points.empty() || extend_distance < 0.1) return output;
  const bool driving_forward =
    autoware::motion_utils::isDrivingForwardWithTwist(input_points).value_or(true);
  const auto & goal = input_points.back();
  for (double distance = step_length; distance < extend_distance - step_length; distance += step_length) {
    output.push_back(extendTrajectoryPoint(distance, goal, driving_forward));
  }
  output.push_back(extendTrajectoryPoint(extend_distance, goal, driving_forward));
  return output;
}

std::vector<autoware_planning_msgs::TrajectoryPoint> decimate_trajectory_points_from_ego(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory_points,
  const geometry_msgs::Pose & current_pose, const double ego_nearest_dist_threshold,
  const double ego_nearest_yaw_threshold, const double decimate_trajectory_step_length,
  const double goal_extended_trajectory_length)
{
  if (trajectory_points.size() < 2U) return trajectory_points;
  const size_t ego_segment = autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    trajectory_points, current_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
  const auto from_ego = std::vector<autoware_planning_msgs::TrajectoryPoint>(
    trajectory_points.begin() + static_cast<std::ptrdiff_t>(ego_segment), trajectory_points.end());
  const auto decimated = resampleTrajectoryPoints(from_ego, decimate_trajectory_step_length);
  const auto extended = get_extended_trajectory_points(
    decimated, goal_extended_trajectory_length, decimate_trajectory_step_length);
  return extended.size() < 2U ? trajectory_points : extended;
}

void insertStop(std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory, const geometry_msgs::Point & stop_point)
{
  if (trajectory.size() < 2) return;
  const auto segment = autoware::motion_utils::findNearestSegmentIndex(trajectory, stop_point);
  const auto index = autoware::motion_utils::insertTargetPoint(segment, stop_point, trajectory, 5e-2);
  if (!index) return;
  for (size_t i = *index; i < trajectory.size(); ++i) trajectory.at(i).longitudinal_velocity_mps = 0.0F;
}

void insertSlowdown(std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory, const SlowdownInterval & interval)
{
  if (trajectory.size() < 2) return;
  const auto from_segment = autoware::motion_utils::findNearestSegmentIndex(trajectory, interval.from);
  const auto from_index = autoware::motion_utils::insertTargetPoint(from_segment, interval.from, trajectory);
  const auto to_segment = autoware::motion_utils::findNearestSegmentIndex(trajectory, interval.to);
  const auto to_index = autoware::motion_utils::insertTargetPoint(to_segment, interval.to, trajectory);
  if (!from_index || !to_index || *from_index > *to_index) return;
  for (size_t i = *from_index; i <= *to_index; ++i) {
    trajectory.at(i).longitudinal_velocity_mps = std::min(trajectory.at(i).longitudinal_velocity_mps, static_cast<float>(interval.velocity));
  }
}

void applyPlanningResult(std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory, const VelocityPlanningResult & result)
{
  for (const auto & stop : result.stop_points) insertStop(trajectory, stop);
  for (const auto & slowdown : result.slowdown_intervals) insertSlowdown(trajectory, slowdown);
}
}  // namespace autoware::motion_velocity_planner::utils
