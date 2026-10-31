#pragma once

#include <geometry_msgs/Pose.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <autoware/behavior_path_planner_common/data_types.hpp>
#include <autoware/behavior_path_planner_common/parameters.hpp>

#include <limits>
#include <memory>

namespace autoware::behavior_path_planner { struct PlannerData; }
namespace autoware::route_handler { class RouteHandler; }

namespace autoware::behavior_path_planner::utils
{

double getSignedDistance(
  const geometry_msgs::Pose & current_pose, const geometry_msgs::Pose & goal_pose,
  const lanelet::ConstLanelets & lanelets);

double getDistanceToEndOfLane(
  const geometry_msgs::Pose & current_pose, const lanelet::ConstLanelets & lanelets);

lanelet::ConstLanelets getCurrentLanes(
  const std::shared_ptr<const PlannerData> & planner_data, double backward_path_length,
  double forward_path_length);

lanelet::ConstLanelets getCurrentLanes(const std::shared_ptr<const PlannerData> & planner_data);

lanelet::ConstLanelets getCurrentLanesFromPath(
  const autoware_internal_planning_msgs::PathWithLaneId & path,
  const std::shared_ptr<const PlannerData> & planner_data);

lanelet::ConstLanelets calcLaneAroundPose(
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const geometry_msgs::Pose & pose, double forward_length, double backward_length,
  double distance_threshold = std::numeric_limits<double>::max(),
  double yaw_threshold = std::numeric_limits<double>::max());

autoware_internal_planning_msgs::PathWithLaneId getCenterLinePath(
  const autoware::route_handler::RouteHandler & route_handler,
  const lanelet::ConstLanelets & lanelet_sequence, const geometry_msgs::Pose & pose,
  double backward_path_length, double forward_path_length,
  const BehaviorPathPlannerParameters & parameters);

// Query an ego-relative window without making its moving crop endpoints into
// spline knots. The input must retain the original centerline/waypoint support.
autoware_internal_planning_msgs::PathWithLaneId resampleCenterLinePathWithSupport(
  const autoware_internal_planning_msgs::PathWithLaneId & source,
  const geometry_msgs::Pose & pose, double backward_path_length, double forward_path_length,
  const BehaviorPathPlannerParameters & parameters);

autoware_internal_planning_msgs::PathWithLaneId getCenterLinePathFromLanelet(
  const lanelet::ConstLanelet & lanelet, const std::shared_ptr<const PlannerData> & planner_data);

bool checkPathRelativeAngle(
  const autoware_internal_planning_msgs::PathWithLaneId & path, double angle_threshold);

// One-hop route extension and lane flattening used by the lane-departure gate.
// They intentionally prefer route lanelets, matching the upstream planner
// policy, and never fabricate a successor when the route handler has none.
lanelet::ConstLanelets extendLanes(
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const lanelet::ConstLanelets & lanes);

lanelet::ConstLanelets transformToLanelets(const DrivableLanes & drivable_lanes);
lanelet::ConstLanelets transformToLanelets(const std::vector<DrivableLanes> & drivable_lanes);

}  // namespace autoware::behavior_path_planner::utils
