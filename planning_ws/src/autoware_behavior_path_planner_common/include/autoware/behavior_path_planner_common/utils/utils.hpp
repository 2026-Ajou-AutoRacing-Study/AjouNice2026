#pragma once

// ROS1 umbrella for the upstream common utility surface.  The algorithms that
// already have independent ROS1 implementations stay in their focused headers;
// this file supplies the small geometry bridge which lane-change consumes.
#include <autoware/behavior_path_planner_common/data_manager.hpp>
#include <autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp>
#include <autoware/behavior_path_planner_common/utils/path_utils.hpp>
#include <autoware/behavior_path_planner_common/utils/route_utils.hpp>
#include <autoware/behavior_path_planner_common/utils/object_distance.hpp>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <vector>
#include <optional>
#include <functional>

namespace autoware::behavior_path_planner::utils
{
// Upstream goal refinement falls back to position-only matching when no
// segment satisfies the pose distance/yaw constraints.
template <class Points>
size_t findNearestSegmentIndex(
  const Points & points, const geometry_msgs::Pose & pose,
  double dist_threshold, double yaw_threshold)
{
  const auto index = autoware::motion_utils::findNearestSegmentIndex(
    points, pose, dist_threshold, yaw_threshold);
  if (index) return *index;
  return autoware::motion_utils::findNearestSegmentIndex(points, pose.position);
}
using drivable_area_expansion::DrivableAreaExpansionParameters;
struct FrenetPoint
{
  double length{0.0};
  double distance{0.0};
};

bool checkCollisionBetweenFootprintAndObjects(
  const autoware_utils::LinearRing2d & footprint, const geometry_msgs::Pose & pose,
  const autoware_perception_msgs::PredictedObjects & objects, double margin);
std::optional<lanelet::ConstLanelet> getRightLanelet(
  const lanelet::ConstLanelet & lane, const lanelet::ConstLanelets & lanes);
std::optional<lanelet::ConstLanelet> getLeftLanelet(
  const lanelet::ConstLanelet & lane, const lanelet::ConstLanelets & lanes);
autoware_internal_planning_msgs::PathPointWithLaneId insertStopPoint(
  double length, PathWithLaneId & path);
std::optional<double> calc_feasible_decel_distance(
  const std::shared_ptr<const PlannerData> & data, double acceleration_limit,
  double jerk_limit, double target_velocity);
PoseWithDetailOpt insert_feasible_stop_point(
  PathWithLaneId & path, const std::shared_ptr<const PlannerData> & data,
  double maximum_deceleration, double maximum_jerk, const std::string & stop_reason);
std::vector<DrivableLanes> generateDrivableLanesWithShoulderLanes(
  const lanelet::ConstLanelets & current, const lanelet::ConstLanelets & shoulders);
std::vector<DrivableLanes> getNonOverlappingExpandedLanes(
  PathWithLaneId & path, const std::vector<DrivableLanes> & lanes,
  const DrivableAreaExpansionParameters & parameters);

template <class T>
FrenetPoint convertToFrenetPoint(
  const T & points, const geometry_msgs::Point & search_point, const size_t segment_index)
{
  return {
    autoware::motion_utils::calcSignedArcLength(points, 0, segment_index) +
      autoware::motion_utils::calcLongitudinalOffsetToSegment(points, segment_index, search_point),
    autoware::motion_utils::calcLateralOffset(points, search_point, segment_index)};
}
geometry_msgs::Pose to_geom_msg_pose(
  const lanelet::BasicPoint3d & src_point, const lanelet::ConstLanelet & target_lane);
geometry_msgs::Pose to_geom_msg_pose(
  const lanelet::BasicPoint2d & src_point, const lanelet::ConstLanelet & target_lane);

bool isEgoWithinOriginalLane(
  const lanelet::ConstLanelets & current_lanes, const geometry_msgs::Pose & current_pose,
  const BehaviorPathPlannerParameters & common_param, double outer_margin = 0.0);
bool isEgoWithinOriginalLane(
  const lanelet::BasicPolygon2d & lane_polygon, const geometry_msgs::Pose & current_pose,
  const BehaviorPathPlannerParameters & common_param, double outer_margin = 0.0);

double getSignedDistanceFromLaneBoundary(
  const lanelet::ConstLanelet & lanelet, const geometry_msgs::Point & position, bool left_side);

double getDistanceToNextIntersection(
  const geometry_msgs::Pose & current_pose, const lanelet::ConstLanelets & lanelets);
double getDistanceToCrosswalk(
  const geometry_msgs::Pose & current_pose, const lanelet::ConstLanelets & lanelets,
  const lanelet::routing::RoutingGraphContainer & overall_graphs);
double getArcLengthToTargetLanelet(
  const lanelet::ConstLanelets & lanelet_sequence, const lanelet::ConstLanelet & target_lane,
  const geometry_msgs::Pose & pose);
autoware_utils_geometry::Polygon2d toPolygon2d(const lanelet::ConstLanelet & lanelet);
autoware_utils_geometry::Polygon2d toPolygon2d(const lanelet::BasicPolygon2d & polygon);
std::vector<lanelet::ConstLanelets> getPrecedingLanelets(
  const autoware::route_handler::RouteHandler & route_handler,
  const lanelet::ConstLanelets & target_lanes, const geometry_msgs::Pose & current_pose,
  double backward_length);

// Source-equivalent route-goal policy.  A goal in a shoulder lane remains a
// valid modification target even when the route's explicit flag is disabled.
bool isAllowedGoalModification(
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler);
bool checkOriginalGoalIsInShoulder(
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler);

// Goal-planner provider surface.  These preserve the source geometry policy:
// a goal already in its lanelet is unchanged, while an external goal is
// projected onto the nearest centerline segment with its lane-heading yaw.
geometry_msgs::Pose refineGoal(
  const geometry_msgs::Pose & goal, const lanelet::ConstLanelet & goal_lanelet);
bool isInLanelets(
  const geometry_msgs::Pose & pose, const lanelet::ConstLanelets & lanes);
lanelet::ConstLanelets extendNextLane(
  std::shared_ptr<autoware::route_handler::RouteHandler> route_handler,
  const lanelet::ConstLanelets & lanes, bool only_in_route = false);
lanelet::ConstLanelets extendPrevLane(
  std::shared_ptr<autoware::route_handler::RouteHandler> route_handler,
  const lanelet::ConstLanelets & lanes, bool only_in_route = false);
lanelet::ConstLanelets getExtendedCurrentLanes(const std::shared_ptr<const PlannerData> & data);
lanelet::ConstLanelets getExtendedCurrentLanes(
  const std::shared_ptr<const PlannerData> & data, double backward_length,
  double forward_length, bool forward_only_in_route);
lanelet::ConstLanelets getBackwardLanelets(
  const autoware::route_handler::RouteHandler & route_handler,
  const lanelet::ConstLanelets & lanes, const geometry_msgs::Pose & pose, double backward_length);
lanelet::ConstLanelets combineLanelets(
  const lanelet::ConstLanelets & base, const lanelet::ConstLanelets & added);
double getSignedDistanceFromBoundary(
  const lanelet::ConstLanelets & lanelets, const geometry_msgs::Pose & pose, bool left_side);
std::optional<double> getSignedDistanceFromBoundary(
  const lanelet::ConstLanelets & lanelets, double vehicle_width, double base_link2front,
  double base_link2rear, const geometry_msgs::Pose & vehicle_pose, bool left_side);
lanelet::ConstLanelets getExtendedCurrentLanesFromPath(
  const PathWithLaneId & path, const std::shared_ptr<const PlannerData> & planner_data,
  double backward_length, double forward_length, bool forward_only_in_route);
bool set_goal(
  double search_radius_range, double search_rad_range, double output_path_interval,
  const autoware_internal_planning_msgs::PathWithLaneId & input,
  const geometry_msgs::Pose & goal, int64_t goal_lane_id,
  const std::function<lanelet::ConstLanelet(int64_t)> & get_lanelet_by_id,
  autoware_internal_planning_msgs::PathWithLaneId * output);

autoware_internal_planning_msgs::PathWithLaneId refinePathForGoal(
  double search_radius_range, double search_rad_range, double output_path_interval,
  const autoware_internal_planning_msgs::PathWithLaneId & input,
  const geometry_msgs::Pose & goal, int64_t goal_lane_id,
  const std::function<lanelet::ConstLanelet(int64_t)> & get_lanelet_by_id);

std::optional<double> calc_point_dist_to_closest_lane_boundary(
  const autoware::route_handler::RouteHandler & route_handler,
  const geometry_msgs::Pose & ego_baselink_pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::string & direction);

// Map-backed polygon lookup used by the full drivable-bound expansion path.
std::optional<lanelet::Polygon3d> getPolygonByPoint(
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const lanelet::ConstPoint3d & point, const std::string & polygon_name);
}  // namespace autoware::behavior_path_planner::utils
