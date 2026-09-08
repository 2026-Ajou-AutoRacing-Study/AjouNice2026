#pragma once

#include <autoware_planning_msgs/Trajectory.h>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <geometry_msgs/Pose.h>

#include <cstddef>
#include <optional>

namespace autoware::planning_validator::intersection_collision_checker
{
struct EgoTrajectory
{
  std::vector<autoware_planning_msgs::TrajectoryPoint> front_trajectory;
  std::vector<autoware_planning_msgs::TrajectoryPoint> back_trajectory;
  std::size_t front_index{};
  std::size_t back_index{};
};

// ROS1 rendering of IntersectionCollisionChecker::get_ego_trajectory.
std::optional<EgoTrajectory> make_ego_trajectory(
  const autoware_planning_msgs::Trajectory & resampled_trajectory,
  const geometry_msgs::Pose & ego_pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info);
}  // namespace autoware::planning_validator::intersection_collision_checker
