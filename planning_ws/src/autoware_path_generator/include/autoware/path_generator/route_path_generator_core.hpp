#pragma once

#include <autoware/lanelet2_utils/route_manager.hpp>
#include <autoware/trajectory/utils/reference_path.hpp>

#include <autoware_map_msgs/LaneletMapBin.h>
#include <autoware_planning_msgs/LaneletRoute.h>
#include <autoware_internal_planning_msgs/PathWithLaneId.h>

#include <optional>

namespace autoware::path_generator
{
// Synchronous ROS1 core corresponding to the upstream PathGenerator route-manager
// and reference-path portion. Node scheduling, goal connection and signals remain
// outside this bounded unit.
struct RoutePathGeneratorParameters
{
  double ego_nearest_dist_threshold{3.0};
  double ego_nearest_yaw_threshold{1.046};
  double forward_length{100.0};
  double backward_length{20.0};
};

std::optional<autoware_internal_planning_msgs::PathWithLaneId> generateRoutePath(
  const autoware_map_msgs::LaneletMapBin & map, const autoware_planning_msgs::LaneletRoute & route,
  const geometry_msgs::Pose & initial_pose, const geometry_msgs::Pose & current_pose,
  const RoutePathGeneratorParameters & parameters = {});
}  // namespace autoware::path_generator
