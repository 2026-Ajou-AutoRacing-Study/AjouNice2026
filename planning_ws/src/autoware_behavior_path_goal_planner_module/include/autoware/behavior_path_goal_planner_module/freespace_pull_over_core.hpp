#pragma once

#include <autoware/freespace_planning_algorithms/astar_search.hpp>
#include <autoware/behavior_path_planner_common/utils/path_utils.hpp>
#include <autoware/behavior_path_planner_common/utils/parking_departure/velocity_utils.hpp>
#include <autoware/behavior_path_goal_planner_module/pull_over_path_core.hpp>

#include <optional>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{
struct FreespacePartialPaths
{
  std::vector<autoware_internal_planning_msgs::PathWithLaneId> paths;
  std::vector<std::pair<double, double>> terminal_velocity_and_accel;
};

// ROS1 closure of the upstream FreespacePullOver algorithm ownership boundary.
// PullOverPath construction and GoalPlanner threading deliberately remain
// outside this class.
class FreespacePullOverCore
{
public:
  FreespacePullOverCore(
    const ros::NodeHandle & private_node,
    const autoware::freespace_planning_algorithms::VehicleShape & vehicle_shape);

  void setMap(const nav_msgs::OccupancyGrid & costmap);
  std::optional<autoware::freespace_planning_algorithms::PlannerWaypoints> plan(
    const geometry_msgs::Pose & current_pose, const geometry_msgs::Pose & goal_pose);
  std::optional<autoware_internal_planning_msgs::PathWithLaneId> planPath(
    const geometry_msgs::Pose & current_pose, const geometry_msgs::Pose & goal_pose,
    double velocity, const lanelet::ConstLanelets & lanelets);
  std::optional<FreespacePartialPaths> planForwardPartialPaths(
    const geometry_msgs::Pose & current_pose, const geometry_msgs::Pose & goal_pose,
    double velocity, const lanelet::ConstLanelets & lanelets, double acceleration = 0.0);
  static std::optional<FreespacePartialPaths> prepareForwardPartialPaths(
    const autoware_internal_planning_msgs::PathWithLaneId & path, double velocity,
    double acceleration = 0.0);
  static std::optional<autoware_internal_planning_msgs::PathWithLaneId> completeGoalPath(
    autoware_internal_planning_msgs::PathWithLaneId path,
    const geometry_msgs::Pose & end_pose, const geometry_msgs::Pose & goal_pose, bool use_back);
  std::optional<PullOverPathCore> planCompletedPullOverPath(
    const geometry_msgs::Pose & current_pose, const geometry_msgs::Pose & end_pose,
    const geometry_msgs::Pose & goal_pose, double velocity, const lanelet::ConstLanelets & lanelets,
    size_t goal_id, bool use_back, double acceleration = 0.0);

private:
  autoware::freespace_planning_algorithms::AstarSearch planner_;
  bool has_map_{false};
};
}  // namespace autoware::behavior_path_planner
