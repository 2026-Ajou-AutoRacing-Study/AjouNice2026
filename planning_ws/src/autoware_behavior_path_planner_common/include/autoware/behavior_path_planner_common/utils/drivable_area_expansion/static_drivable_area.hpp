#pragma once

#include <autoware/behavior_path_planner_common/data_types.hpp>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::route_handler { class RouteHandler; }
namespace autoware::behavior_path_planner { struct PlannerData; }

namespace autoware::behavior_path_planner::utils
{

std::optional<size_t> getOverlappedLaneletId(const std::vector<DrivableLanes> & lanes);

std::vector<DrivableLanes> cutOverlappedLanes(
  autoware_internal_planning_msgs::PathWithLaneId & path,
  const std::vector<DrivableLanes> & lanes);

std::vector<DrivableLanes> generateDrivableLanes(const lanelet::ConstLanelets & lanelets);

std::vector<DrivableLanes> expandLanelets(
  const std::vector<DrivableLanes> & drivable_lanes, double left_bound_offset,
  double right_bound_offset, const std::vector<std::string> & types_to_skip = {});

std::vector<lanelet::ConstPoint3d> getBoundWithHatchedRoadMarkings(
  const std::vector<lanelet::ConstPoint3d> & original_bound,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler);

std::vector<lanelet::ConstPoint3d> getBoundWithIntersectionAreas(
  const std::vector<lanelet::ConstPoint3d> & original_bound,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const std::vector<DrivableLanes> & drivable_lanes, bool is_left,
  double lanelet_overlap_ratio_threshold = 0.9);

std::pair<std::vector<lanelet::ConstPoint3d>, bool> getBoundWithFreeSpaceAreas(
  const std::vector<lanelet::ConstPoint3d> & original_bound,
  const std::vector<lanelet::ConstPoint3d> & other_side_bound,
  const std::shared_ptr<const PlannerData> planner_data, bool is_left);

std::vector<geometry_msgs::Point> postProcess(
  const std::vector<geometry_msgs::Point> & original_bound,
  const autoware_internal_planning_msgs::PathWithLaneId & path,
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<DrivableLanes> & drivable_lanes, bool is_left,
  bool is_driving_forward = true);

std::vector<geometry_msgs::Point> calcBound(
  const autoware_internal_planning_msgs::PathWithLaneId & path,
  const std::shared_ptr<const PlannerData> planner_data,
  const std::vector<DrivableLanes> & drivable_lanes,
  bool enable_expanding_hatched_road_markings, bool enable_expanding_intersection_areas,
  bool enable_expanding_freespace_areas, bool is_left, bool is_driving_forward = true,
  double intersection_area_lanelet_overlap_ratio_threshold = 0.9);

void extractObstaclesFromDrivableArea(
  autoware_internal_planning_msgs::PathWithLaneId & path,
  const std::vector<DrivableAreaInfo::Obstacle> & obstacles);

std::vector<DrivableLanes> combineDrivableLanes(
  const std::vector<DrivableLanes> & original_drivable_lanes_vec,
  const std::vector<DrivableLanes> & new_drivable_lanes_vec);

DrivableAreaInfo combineDrivableAreaInfo(
  const DrivableAreaInfo & first, const DrivableAreaInfo & second);

}  // namespace autoware::behavior_path_planner::utils
