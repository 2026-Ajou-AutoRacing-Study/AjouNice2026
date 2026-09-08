#ifndef AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PATH_UTILS_HPP_
#define AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PATH_UTILS_HPP_

#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <autoware/freespace_planning_algorithms/abstract_algorithm.hpp>
#include <autoware/behavior_path_planner_common/planner_data.hpp>
#include <autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp>
#include <geometry_msgs/Pose.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

namespace autoware::behavior_path_planner::utils
{
std::vector<double> calcPathArcLengthArray(
  const autoware_internal_planning_msgs::PathWithLaneId & path, size_t start = 0,
  size_t end = std::numeric_limits<size_t>::max(), double offset = 0.0);
autoware_internal_planning_msgs::PathWithLaneId resamplePathWithSpline(
  const autoware_internal_planning_msgs::PathWithLaneId & path, double interval,
  bool keep_input_points = false,
  std::pair<double, double> target_section = {0.0, std::numeric_limits<double>::max()});
size_t getIdxByArclength(const autoware_internal_planning_msgs::PathWithLaneId & path, size_t target_idx, double signed_arc);
void clipPathLength(autoware_internal_planning_msgs::PathWithLaneId & path, size_t target_idx, double forward, double backward);
std::vector<size_t> getReversingIndices(const autoware_internal_planning_msgs::PathWithLaneId & path);
std::vector<autoware_internal_planning_msgs::PathWithLaneId> dividePath(
  const autoware_internal_planning_msgs::PathWithLaneId & path, const std::vector<size_t> & indices);
autoware_internal_planning_msgs::PathWithLaneId convertWayPointsToPathWithLaneId(
  const autoware::freespace_planning_algorithms::PlannerWaypoints & waypoints,
  double velocity, const lanelet::ConstLanelets & lanelets);
void correctDividedPathVelocity(std::vector<autoware_internal_planning_msgs::PathWithLaneId> & paths);
std::vector<double> spline_two_points(const std::vector<double> & base_s, const std::vector<double> & base_x,
  double begin_diff, double end_diff, const std::vector<double> & new_s);
std::vector<geometry_msgs::Pose> interpolatePose(
  const geometry_msgs::Pose & start_pose, const geometry_msgs::Pose & end_pose, double resample_interval);
geometry_msgs::Pose getUnshiftedEgoPose(
  const geometry_msgs::Pose & ego_pose,
  const autoware::behavior_path_planner::ShiftedPath & previous_path);
autoware_internal_planning_msgs::PathWithLaneId calcCenterLinePath(
  const std::shared_ptr<const autoware::behavior_path_planner::PlannerData> & planner_data,
  const geometry_msgs::Pose & reference_pose, double longest_distance_to_shift_line,
  const std::optional<autoware_internal_planning_msgs::PathWithLaneId> & previous_module_path = std::nullopt);
autoware_internal_planning_msgs::PathWithLaneId combinePath(
  const autoware_internal_planning_msgs::PathWithLaneId & path1,
  const autoware_internal_planning_msgs::PathWithLaneId & path2);
}  // namespace autoware::behavior_path_planner::utils

#endif
