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
#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

namespace autoware::behavior_velocity_planner
{
using TrajectoryPoints = std::vector<autoware_planning_msgs::TrajectoryPoint>;

namespace
{
bool applyExternalLimit(
  TrajectoryPoints & trajectory, const geometry_msgs::Pose & pose, const double v0,
  const double a0, const PlannerData & data)
{
  if (!data.external_velocity_limit) return true;
  const auto & limit = *data.external_velocity_limit;
  const auto & smoother = *data.velocity_smoother_;
  const double target = limit.max_velocity;
  const double min_acc = limit.use_constraints
    ? limit.constraints.min_acceleration : smoother.getMinDecel();
  const double max_jerk = limit.use_constraints
    ? limit.constraints.max_jerk : smoother.getMaxJerk();
  const double min_jerk = limit.use_constraints
    ? limit.constraints.min_jerk : smoother.getMinJerk();
  if (
    trajectory.size() < 2 || !std::isfinite(v0) || v0 < 0.0 || !std::isfinite(a0) ||
    !std::isfinite(target) || target < 0.0 || !std::isfinite(min_acc) || min_acc >= 0.0 ||
    !std::isfinite(max_jerk) || max_jerk <= 0.0 ||
    !std::isfinite(min_jerk) || min_jerk >= 0.0) {
    ROS_WARN_THROTTLE(1.0, "[BVP] Cannot predict external velocity limit: invalid motion/constraints");
    return false;
  }

  // Same jerk-constrained distance and speed ceiling as the final smoother,
  // but recomputed from measured motion for this prediction (no previous-output state).
  double ceiling = a0 > 0.0 ? v0 - 0.5 * a0 * a0 / min_jerk : v0;
  double distance = 0.0;
  if (target < ceiling) {
    std::map<double, double> jerk_profile;
    if (!autoware::velocity_smoother::trajectory_utils::calcStopDistWithJerkConstraints(
          v0, a0, max_jerk, min_jerk, min_acc, target, jerk_profile, distance) ||
        !std::isfinite(distance) || distance < 0.0) {
      ROS_WARN_THROTTLE(1.0, "[BVP] External velocity limit braking-distance prediction failed");
      return false;
    }
    // Matches margin_to_insert_external_velocity_limit in both current ROS1 profiles.
    constexpr double insertion_margin = 0.3;
    distance += insertion_margin;
  } else {
    ceiling = target;
  }
  autoware::velocity_smoother::trajectory_utils::applyMaximumVelocityLimit(
    0, trajectory.size(), ceiling, trajectory);
  const auto segment = autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    trajectory, pose, data.ego_nearest_dist_threshold, data.ego_nearest_yaw_threshold);
  const auto inserted = autoware::motion_utils::insertTargetPoint(segment, distance, trajectory);
  if (inserted) {
    autoware::velocity_smoother::trajectory_utils::applyMaximumVelocityLimit(
      *inserted, trajectory.size(), target, trajectory);
  } else {
    // The target is beyond this path's horizon. Preserve the current-speed prefix,
    // as the final smoother does, instead of clamping the entire path immediately.
    trajectory.back().longitudinal_velocity_mps = std::min<double>(
      trajectory.back().longitudinal_velocity_mps, target);
  }
  return true;
}
}  // namespace

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
  auto trajectory = autoware::motion_utils::convertToTrajectoryPoints(in_path);
  // Apply the external target at its braking distance BEFORE geometry filters
  // and longitudinal smoothing. A post-smoothing clamp destroys the motion profile.
  if (!applyExternalLimit(trajectory, current_pose, v0, a0, planner_data)) return false;
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
  out_path = autoware::motion_utils::convertToPathWithLaneId(smoothed);
  return true;
}
}  // namespace autoware::behavior_velocity_planner
