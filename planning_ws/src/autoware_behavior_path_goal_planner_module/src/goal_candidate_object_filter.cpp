#include <autoware/behavior_path_goal_planner_module/goal_candidate_object_filter.hpp>

#include <autoware/behavior_path_planner_common/utils/object_distance.hpp>

namespace autoware::behavior_path_planner
{
autoware_perception_msgs::PredictedObjects filterGoalCandidateObjectsByLateralDistance(
  const geometry_msgs::Pose & ego_pose, const double vehicle_width,
  const autoware_perception_msgs::PredictedObjects & objects, const double distance_threshold,
  const bool filter_inside)
{
  autoware_perception_msgs::PredictedObjects filtered_objects;
  for (const auto & object : objects.objects) {
    const double distance =
      utils::calcLateralDistanceFromEgoToObject(ego_pose, vehicle_width, object);
    if (filter_inside ? distance < distance_threshold : distance > distance_threshold) {
      filtered_objects.objects.push_back(object);
    }
  }
  return filtered_objects;
}
}  // namespace autoware::behavior_path_planner
