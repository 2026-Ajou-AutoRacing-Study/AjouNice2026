#include <autoware/behavior_path_planner_common/utils/parking_departure/velocity_utils.hpp>

#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <cassert>
#include <cmath>

namespace autoware::behavior_path_planner::utils::parking_departure
{

void modifyVelocityByDirection(
  std::vector<autoware_internal_planning_msgs::PathWithLaneId> & paths,
  std::vector<std::pair<double, double>> & terminal_vel_acc_pairs,
  const double target_velocity, const double acceleration)
{
  assert(paths.size() == terminal_vel_acc_pairs.size());
  for (size_t index = 0; index < paths.size(); ++index) {
    auto & path = paths.at(index);
    const auto is_driving_forward = autoware::motion_utils::isDrivingForward(path.points);
    if (!is_driving_forward) {
      terminal_vel_acc_pairs.at(index) = {0.0, 0.0};
      continue;
    }
    const double sign = *is_driving_forward ? 1.0 : -1.0;
    for (auto & point : path.points) {
      point.point.longitudinal_velocity_mps = static_cast<float>(
        sign * std::abs(point.point.longitudinal_velocity_mps));
    }
    terminal_vel_acc_pairs.at(index) = {sign * target_velocity, sign * acceleration};
    path.points.back().point.longitudinal_velocity_mps = 0.0F;
  }
}

std::pair<double, double> getPairsTerminalVelocityAndAccel(
  const std::vector<std::pair<double, double>> & terminal_vel_acc_pairs,
  const size_t current_path_idx)
{
  if (terminal_vel_acc_pairs.size() <= current_path_idx) return {0.0, 0.0};
  return terminal_vel_acc_pairs.at(current_path_idx);
}

}  // namespace autoware::behavior_path_planner::utils::parking_departure
