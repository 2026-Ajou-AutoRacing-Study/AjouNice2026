#ifndef AUTOWARE_PLANNING_VALIDATOR__INVALID_TRAJECTORY_HANDLING_CORE_HPP_
#define AUTOWARE_PLANNING_VALIDATOR__INVALID_TRAJECTORY_HANDLING_CORE_HPP_

#include <autoware_planning_msgs/Trajectory.h>

#include <optional>

namespace autoware::planning_validator
{
enum class InvalidTrajectoryHandlingType : int
{
  PublishAsIs = 0,
  UsePreviousResult = 1,
  UsePreviousResultWithSoftStop = 2,
};

struct InvalidTrajectoryHandlingParameters
{
  InvalidTrajectoryHandlingType default_handling_type{InvalidTrajectoryHandlingType::PublishAsIs};
  double soft_stop_deceleration{-1.0};
  double soft_stop_jerk_lim{0.3};
};

// Returns no trajectory only when the selected source policy requires a
// previous valid trajectory that is not available.  Soft-stop construction is
// deliberately separate so the node can retain the source's cached stop path.
std::optional<autoware_planning_msgs::Trajectory> select_invalid_trajectory(
  const autoware_planning_msgs::Trajectory & current,
  const autoware_planning_msgs::Trajectory * last_valid,
  InvalidTrajectoryHandlingType handling);

autoware_planning_msgs::Trajectory make_soft_stop_trajectory(
  const autoware_planning_msgs::Trajectory & last_valid, size_t nearest_index,
  double current_velocity, double current_acceleration,
  const InvalidTrajectoryHandlingParameters & parameters);
}  // namespace autoware::planning_validator

#endif
