#include <autoware/behavior_path_goal_planner_module/freespace_pull_over_core.hpp>

#include <autoware/motion_utils/trajectory/trajectory.hpp>

namespace autoware::behavior_path_planner
{
namespace
{
autoware::freespace_planning_algorithms::PlannerCommonParam makePlannerCommonParam(
  const ros::NodeHandle & node)
{
  // Preserve the full source Goal Planner freespace contract.  Leaving the
  // remaining aggregate members value-initialized changes A* behavior (most
  // notably its turn and goal-reachability model) even though it compiles.
  // In particular theta_size and turning_steps must never remain zero.
  autoware::freespace_planning_algorithms::PlannerCommonParam parameters{};
  node.param("time_limit", parameters.time_limit, 3000.0);
  node.param("theta_size", parameters.theta_size, 120);
  node.param("curve_weight", parameters.curve_weight, 0.5);
  node.param("reverse_weight", parameters.reverse_weight, 1.0);
  node.param("direction_change_weight", parameters.direction_change_weight, 1.5);
  node.param("lateral_goal_range", parameters.lateral_goal_range, 0.5);
  node.param("longitudinal_goal_range", parameters.longitudinal_goal_range, 2.0);
  node.param("angle_goal_range", parameters.angle_goal_range, 6.0);
  node.param("max_turning_ratio", parameters.max_turning_ratio, 0.5);
  node.param("turning_steps", parameters.turning_steps, 1);
  node.param("obstacle_threshold", parameters.obstacle_threshold, 30);
  return parameters;
}
}  // namespace

FreespacePullOverCore::FreespacePullOverCore(
  const ros::NodeHandle & private_node,
  const autoware::freespace_planning_algorithms::VehicleShape & vehicle_shape)
: planner_(makePlannerCommonParam(private_node), vehicle_shape, private_node)
{
}

void FreespacePullOverCore::setMap(const nav_msgs::OccupancyGrid & costmap)
{
  planner_.setMap(costmap);
  has_map_ = true;
}

std::optional<autoware::freespace_planning_algorithms::PlannerWaypoints>
FreespacePullOverCore::plan(
  const geometry_msgs::Pose & current_pose, const geometry_msgs::Pose & goal_pose)
{
  if (!has_map_) return std::nullopt;
  try {
    if (!planner_.makePlan(current_pose, goal_pose)) return std::nullopt;
  } catch (const std::exception &) {
    return std::nullopt;
  }
  return planner_.getWaypoints();
}

std::optional<autoware_internal_planning_msgs::PathWithLaneId> FreespacePullOverCore::planPath(
  const geometry_msgs::Pose & current_pose, const geometry_msgs::Pose & goal_pose,
  const double velocity, const lanelet::ConstLanelets & lanelets)
{
  const auto waypoints = plan(current_pose, goal_pose);
  if (!waypoints) return std::nullopt;
  return utils::convertWayPointsToPathWithLaneId(*waypoints, velocity, lanelets);
}

std::optional<FreespacePartialPaths> FreespacePullOverCore::planForwardPartialPaths(
  const geometry_msgs::Pose & current_pose, const geometry_msgs::Pose & goal_pose,
  const double velocity, const lanelet::ConstLanelets & lanelets, const double acceleration)
{
  const auto path = planPath(current_pose, goal_pose, velocity, lanelets);
  if (!path) return std::nullopt;
  return prepareForwardPartialPaths(*path, velocity, acceleration);
}

std::optional<FreespacePartialPaths> FreespacePullOverCore::prepareForwardPartialPaths(
  const autoware_internal_planning_msgs::PathWithLaneId & path, const double velocity,
  const double acceleration)
{
  if (path.points.empty()) return std::nullopt;
  FreespacePartialPaths output;
  output.paths = utils::dividePath(path, utils::getReversingIndices(path));
  output.terminal_velocity_and_accel.resize(output.paths.size());
  utils::parking_departure::modifyVelocityByDirection(
    output.paths, output.terminal_velocity_and_accel, velocity, acceleration);
  for (const auto & partial_path : output.paths) {
    if (!autoware::motion_utils::isDrivingForward(partial_path.points)) return std::nullopt;
  }
  return output;
}

std::optional<autoware_internal_planning_msgs::PathWithLaneId>
FreespacePullOverCore::completeGoalPath(
  autoware_internal_planning_msgs::PathWithLaneId path, const geometry_msgs::Pose & end_pose,
  const geometry_msgs::Pose & goal_pose, const bool use_back)
{
  if (path.points.empty()) return std::nullopt;
  constexpr double goal_distance_threshold = 1.0;
  for (auto it = path.points.begin(); it != path.points.end(); ++it) {
    if (it == path.points.begin()) continue;
    const double dx = end_pose.position.x - it->point.pose.position.x;
    const double dy = end_pose.position.y - it->point.pose.position.y;
    if (std::hypot(dx, dy) < goal_distance_threshold) {
      path.points.erase(it, path.points.end());
      break;
    }
  }
  if (path.points.empty()) return std::nullopt;
  const auto append_pose = [&path](const geometry_msgs::Pose & pose) {
    auto point = path.points.back();
    point.point.pose = pose;
    path.points.push_back(point);
  };
  if (use_back) {
    append_pose(end_pose);
  } else {
    for (const auto & pose : utils::interpolatePose(path.points.back().point.pose, end_pose, 0.5)) {
      append_pose(pose);
    }
    append_pose(end_pose);
    for (const auto & pose : utils::interpolatePose(end_pose, goal_pose, 0.5)) {
      append_pose(pose);
    }
    append_pose(goal_pose);
  }
  return path;
}

std::optional<PullOverPathCore> FreespacePullOverCore::planCompletedPullOverPath(
  const geometry_msgs::Pose & current_pose, const geometry_msgs::Pose & end_pose,
  const geometry_msgs::Pose & goal_pose, const double velocity,
  const lanelet::ConstLanelets & lanelets, const size_t goal_id, const bool use_back,
  const double acceleration)
{
  const auto raw_path = planPath(current_pose, end_pose, velocity, lanelets);
  if (!raw_path) return std::nullopt;
  const auto completed_path = completeGoalPath(*raw_path, end_pose, goal_pose, use_back);
  if (!completed_path) return std::nullopt;
  const auto partial_paths = prepareForwardPartialPaths(*completed_path, velocity, acceleration);
  if (!partial_paths) return std::nullopt;
  GoalCandidateCore goal;
  goal.id = goal_id;
  goal.goal_pose = goal_pose;
  return PullOverPathCore::create(
    partial_paths->paths, current_pose, goal, partial_paths->terminal_velocity_and_accel);
}
}  // namespace autoware::behavior_path_planner
