#include <autoware/behavior_path_goal_planner_module/goal_candidate_safety_core.hpp>
#include <autoware/behavior_path_goal_planner_module/goal_candidate_object_filter.hpp>

#include <autoware/behavior_path_planner_common/utils/object_distance.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <algorithm>

namespace autoware::behavior_path_planner
{
bool checkGoalCandidateCollision(
  const autoware_utils_geometry::LinearRing2d & vehicle_footprint,
  const geometry_msgs::Pose & pose, const autoware_perception_msgs::PredictedObjects & objects,
  const GoalCandidateSafetyParameters & parameters,
  const std::shared_ptr<OccupancyGridBasedCollisionDetector> & occupancy_grid_map)
{
  if (parameters.use_occupancy_grid_for_goal_search) {
    const auto pose_grid_coords = global2local(occupancy_grid_map->getMap(), pose);
    const auto index = pose2index(
      occupancy_grid_map->getMap(), pose_grid_coords, occupancy_grid_map->getParam().theta_size);
    constexpr bool check_out_of_range = false;
    if (occupancy_grid_map->detectCollision(index, check_out_of_range)) return true;
  }

  return utils::checkCollisionBetweenFootprintAndObjects(
    vehicle_footprint, pose, objects, parameters.object_recognition_collision_check_hard_margin);
}

bool checkGoalCandidateLongitudinalMargin(
  const geometry_msgs::Pose & pose, const autoware_perception_msgs::PredictedObjects & objects,
  const GoalCandidateSafetyParameters & parameters,
  const std::shared_ptr<OccupancyGridBasedCollisionDetector> & occupancy_grid_map)
{
  if (
    parameters.use_occupancy_grid_for_goal_search &&
    parameters.use_occupancy_grid_for_goal_longitudinal_margin) {
    constexpr bool check_out_of_range = false;
    const double offset = std::max(
      parameters.longitudinal_margin - parameters.occupancy_grid_collision_check_margin, 0.0);
    for (const double direction : {1.0, -1.0}) {
      const auto offset_pose = autoware_utils_geometry::calc_offset_pose(pose, direction * offset, 0.0, 0.0);
      const auto local_pose = global2local(occupancy_grid_map->getMap(), offset_pose);
      const auto index = pose2index(
        occupancy_grid_map->getMap(), local_pose, occupancy_grid_map->getParam().theta_size);
      if (occupancy_grid_map->detectCollision(index, check_out_of_range)) return true;
    }
  }

  return utils::calcLongitudinalDistanceFromEgoToObjects(
           pose, parameters.base_link2front, parameters.base_link2rear, objects) <
         parameters.longitudinal_margin;
}

void updateGoalCandidateSafety(
  GoalCandidates * candidates, const autoware_utils_geometry::LinearRing2d & vehicle_footprint,
  const autoware_perception_msgs::PredictedObjects & objects,
  const GoalCandidateSafetyParameters & parameters,
  const std::shared_ptr<OccupancyGridBasedCollisionDetector> & occupancy_grid_map)
{
  if (!candidates) return;
  for (auto & candidate : *candidates) {
    constexpr bool filter_inside = true;
    const auto target_objects = filterGoalCandidateObjectsByLateralDistance(
      candidate.goal_pose, parameters.vehicle_width, objects,
      parameters.object_recognition_collision_check_hard_margin, filter_inside);
    if (checkGoalCandidateCollision(
          vehicle_footprint, candidate.goal_pose, objects, parameters, occupancy_grid_map) ||
        checkGoalCandidateLongitudinalMargin(
          candidate.goal_pose, target_objects, parameters, occupancy_grid_map)) {
      candidate.is_safe = false;
      continue;
    }
    candidate.is_safe = true;
  }
}
}  // namespace autoware::behavior_path_planner
