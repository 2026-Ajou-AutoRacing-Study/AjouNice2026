#ifndef AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_AREA_FILTER_HPP_
#define AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_AREA_FILTER_HPP_

#include <autoware_utils_geometry/boost_geometry.hpp>
#include <geometry_msgs/Pose.h>
#include <lanelet2_core/primitives/Polygon.h>
#include <vector>

namespace autoware::behavior_path_planner
{
struct GoalCandidateAreaConstraints
{
  bool use_bus_stop_area{false};
  // ROS1 lanelet BasicPolygon2d is not a Boost Geometry model. Callers must
  // convert its points into this explicitly supported polygon ABI.
  std::vector<autoware_utils_geometry::Polygon2d> bus_stop_areas{};
  std::vector<autoware_utils_geometry::Polygon2d> no_parking_areas{};
  std::vector<autoware_utils_geometry::Polygon2d> no_stopping_areas{};
  autoware_utils_geometry::Polygon2d departure_check_area{};
};

// Source GoalSearcher area order: bus-stop containment, then no-parking and
// no-stopping intersection rejection, then departure-area containment.
bool isGoalCandidatePoseAllowedByAreas(
  const autoware_utils_geometry::LinearRing2d & local_vehicle_footprint,
  const geometry_msgs::Pose & goal_pose, const GoalCandidateAreaConstraints & constraints);

autoware_utils_geometry::Polygon2d toGoalCandidateAreaPolygon(
  const lanelet::ConstPolygon3d & polygon);

GoalCandidateAreaConstraints appendGoalCandidateRegulatoryAreas(
  GoalCandidateAreaConstraints constraints, const lanelet::ConstLanelets & pull_over_lanes);

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_AREA_FILTER_HPP_
