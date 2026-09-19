#include <autoware/behavior_path_goal_planner_module/goal_candidate_object_counter.hpp>

#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <boost/geometry/algorithms/distance.hpp>

namespace autoware::behavior_path_planner
{
void countGoalCandidateObjectsToAvoid(
  GoalCandidates * candidates, const autoware_perception_msgs::PredictedObjects & objects,
  const lanelet::ConstLanelets & current_lanes,
  const autoware_internal_planning_msgs::PathWithLaneId & current_center_line_path,
  const autoware_utils_geometry::LinearRing2d & local_vehicle_footprint,
  const double object_collision_margin)
{
  if (!candidates || current_lanes.empty()) return;
  for (auto & candidate : *candidates) candidate.num_objects_to_avoid = 0;
  for (const auto & object : objects.objects) {
    for (const auto & point : current_center_line_path.points) {
      const auto footprint = autoware_utils_geometry::transform_vector(
        local_vehicle_footprint, autoware_utils_geometry::pose2transform(point.point.pose));
      const auto object_polygon = autoware_utils_geometry::to_polygon2d(object);
      if (boost::geometry::distance(object_polygon, footprint) > object_collision_margin) continue;
      const double object_s =
        autoware::experimental::lanelet2_utils::get_arc_coordinates(
          current_lanes, object.kinematics.initial_pose_with_covariance.pose).length;
      for (auto & candidate : *candidates) {
        const double goal_s = autoware::experimental::lanelet2_utils::get_arc_coordinates(
          current_lanes, candidate.goal_pose).length;
        if (object_s < goal_s) ++candidate.num_objects_to_avoid;
      }
      break;
    }
  }
}
}  // namespace autoware::behavior_path_planner
