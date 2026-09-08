#include <autoware/behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp>

#include <autoware/behavior_path_planner_common/utils/utils.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <algorithm>
#include <limits>

namespace autoware::behavior_path_planner::utils::path_safety_checker
{
std::vector<PoseWithVelocityStamped> createPredictedPath(
  const std::shared_ptr<EgoPredictedPathParams> & ego_predicted_path_params,
  const std::vector<PathPointWithLaneId> & path_points,
  const geometry_msgs::Pose & vehicle_pose, const double current_velocity,
  const size_t ego_seg_idx, const bool is_object_front, const bool limit_to_max_velocity)
{
  if (!ego_predicted_path_params || path_points.empty() ||
      ego_predicted_path_params->time_resolution <= 0.0) {
    return {};
  }

  const double min_velocity = ego_predicted_path_params->min_velocity;
  const double acceleration = ego_predicted_path_params->acceleration;
  const double max_velocity = limit_to_max_velocity ? ego_predicted_path_params->max_velocity
                                                     : std::numeric_limits<double>::infinity();
  const double time_horizon = is_object_front
    ? ego_predicted_path_params->time_horizon_for_front_object
    : ego_predicted_path_params->time_horizon_for_rear_object;
  if (time_horizon <= 0.0) return {};

  const double time_resolution = ego_predicted_path_params->time_resolution;
  const double delay_until_departure = ego_predicted_path_params->delay_until_departure;
  // ROS1 motion-utils exposes the equivalent operation as an offset from the
  // current pose rather than ROS2's absolute-arc-length interpolation.  The
  // segment index is still bounded here to preserve the caller's validated
  // path-location precondition.
  const size_t bounded_ego_seg_idx = std::min(ego_seg_idx, path_points.size() - 1);
  (void)bounded_ego_seg_idx;

  std::vector<PoseWithVelocityStamped> predicted_path;
  for (double time = 0.0; time < time_horizon; time += time_resolution) {
    double velocity = 0.0;
    double length = 0.0;
    if (time >= delay_until_departure) {
      const double elapsed = time - delay_until_departure;
      velocity = std::clamp(
        current_velocity + acceleration * elapsed, min_velocity, max_velocity);
      length = current_velocity * elapsed + 0.5 * acceleration * elapsed * elapsed;
    }
    const auto pose = autoware::motion_utils::calcLongitudinalOffsetPose(
      path_points, vehicle_pose.position, length);
    if (!pose) break;
    predicted_path.emplace_back(time, *pose, velocity);
  }
  return predicted_path;
}
}  // namespace autoware::behavior_path_planner::utils::path_safety_checker
