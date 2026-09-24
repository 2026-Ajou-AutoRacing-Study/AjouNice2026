#include <autoware/behavior_velocity_planner_common/utilization/trajectory_utils.hpp>

#include <autoware/motion_utils/trajectory/conversion.hpp>
#ifdef RCLCPP_DEBUG
#undef RCLCPP_DEBUG
#endif
#ifdef RCLCPP_WARN
#undef RCLCPP_WARN
#endif
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/velocity_smoother/smoother/smoother_base.hpp>
#include <autoware/velocity_smoother/trajectory_utils.hpp>

#include <iostream>
#include <vector>

namespace autoware::behavior_velocity_planner
{
using TrajectoryPoints = std::vector<autoware_planning_msgs::TrajectoryPoint>;

bool smoothPath(
  const autoware_internal_planning_msgs::PathWithLaneId & in_path,
  autoware_internal_planning_msgs::PathWithLaneId & out_path,
  const std::shared_ptr<const PlannerData> & planner_data)
{
  return planner_data && smoothPath(in_path, out_path, *planner_data);
}

bool smoothPath(
  const autoware_internal_planning_msgs::PathWithLaneId & in_path,
  autoware_internal_planning_msgs::PathWithLaneId & out_path, const PlannerData & planner_data)
{
  if (!planner_data.current_odometry || !planner_data.current_velocity ||
      !planner_data.current_acceleration || !planner_data.velocity_smoother_) {
    std::cerr << "[behavior_velocity][trajectory_utils]: missing smoothing input" << std::endl;
    return false;
  }
  const auto & current_pose = planner_data.current_odometry->pose;
  const double v0 = planner_data.current_velocity->twist.linear.x;
  const double a0 = planner_data.current_acceleration->accel.accel.linear.x;
  const auto trajectory = autoware::motion_utils::convertToTrajectoryPoints(in_path);
  const auto lateral = planner_data.velocity_smoother_->applyLateralAccelerationFilter(trajectory);
  const auto steering = planner_data.velocity_smoother_->applySteeringRateLimit(
    lateral, false, planner_data.velocity_smoother_->getBaseParam().sample_ds);
  const auto resampled = planner_data.velocity_smoother_->resampleTrajectory(
    steering, v0, current_pose, planner_data.ego_nearest_dist_threshold,
    planner_data.ego_nearest_yaw_threshold);
  if (resampled.empty()) return false;
  const auto closest = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    resampled, current_pose, planner_data.ego_nearest_dist_threshold,
    planner_data.ego_nearest_yaw_threshold);
  TrajectoryPoints clipped(resampled.begin() + static_cast<std::ptrdiff_t>(closest), resampled.end());
  TrajectoryPoints smoothed;
  std::vector<TrajectoryPoints> debug;
  if (!planner_data.velocity_smoother_->apply(v0, a0, clipped, smoothed, debug, false)) return false;
  smoothed.insert(smoothed.begin(), resampled.begin(), resampled.begin() + static_cast<std::ptrdiff_t>(closest));
  if (planner_data.external_velocity_limit) {
    autoware::velocity_smoother::trajectory_utils::applyMaximumVelocityLimit(
      closest, smoothed.size(), planner_data.external_velocity_limit->max_velocity, smoothed);
  }
  out_path = autoware::motion_utils::convertToPathWithLaneId(smoothed);
  return true;
}
}  // namespace autoware::behavior_velocity_planner
