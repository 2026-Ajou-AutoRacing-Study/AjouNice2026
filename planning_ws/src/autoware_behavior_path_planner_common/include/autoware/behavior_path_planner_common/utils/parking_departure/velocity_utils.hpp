#ifndef AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PARKING_DEPARTURE__VELOCITY_UTILS_HPP_
#define AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PARKING_DEPARTURE__VELOCITY_UTILS_HPP_

#include <autoware_internal_planning_msgs/PathWithLaneId.h>

#include <cstddef>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner::utils::parking_departure
{

// ROS1 subset of parking_departure/utils.cpp.  Feasible-stop generation and
// safety-check data remain separate provider-bound work.
void modifyVelocityByDirection(
  std::vector<autoware_internal_planning_msgs::PathWithLaneId> & paths,
  std::vector<std::pair<double, double>> & terminal_vel_acc_pairs, double target_velocity,
  double acceleration);

std::pair<double, double> getPairsTerminalVelocityAndAccel(
  const std::vector<std::pair<double, double>> & terminal_vel_acc_pairs,
  size_t current_path_idx);

}  // namespace autoware::behavior_path_planner::utils::parking_departure

#endif
