#pragma once
#include <autoware/route_handler/route_handler.hpp>
#include <memory>
#include <optional>

namespace autoware::behavior_path_planner {
std::optional<lanelet::ConstLanelets> selectPullOverLanes(
  const std::shared_ptr<const autoware::route_handler::RouteHandler> & route_handler,
  const geometry_msgs::Pose & goal_pose, bool left_side, double backward_distance,
  double forward_distance);
}
