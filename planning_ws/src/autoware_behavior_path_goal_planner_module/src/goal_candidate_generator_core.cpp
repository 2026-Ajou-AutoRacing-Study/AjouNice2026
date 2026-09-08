#include <autoware/behavior_path_goal_planner_module/goal_candidate_generator_core.hpp>

#include <autoware_utils_geometry/geometry.hpp>

#include <boost/geometry/algorithms/intersects.hpp>

namespace autoware::behavior_path_planner
{
GoalCandidates generateGoalCandidatesFromSeeds(
  const std::vector<GoalCandidateSeed> & seeds, const bool left_side_parking,
  const double max_lateral_offset, const double lateral_offset_interval,
  const autoware_utils_geometry::LinearRing2d & local_vehicle_footprint,
  const GoalCandidateAreaConstraints & area_constraints)
{
  GoalCandidates candidates;
  if (lateral_offset_interval <= 0.0 || max_lateral_offset < 0.0) return candidates;

  const double sign = left_side_parking ? -1.0 : 1.0;
  std::size_t goal_id = 0;
  for (const auto & seed : seeds) {
    for (double dy = 0.0; dy <= max_lateral_offset; dy += lateral_offset_interval) {
      const auto goal_pose = autoware_utils_geometry::calc_offset_pose(
        seed.pose_with_min_boundary_margin, 0.0, sign * dy, 0.0);
      if (!isGoalCandidatePoseAllowedByAreas(
            local_vehicle_footprint, goal_pose, area_constraints)) {
        // Candidate generation only breaks in the source for no-parking/no-
        // stopping areas. The precomputed area adapter preserves that signal
        // by testing those conditions independently below.
        const auto transformed = autoware_utils_geometry::transform_vector(
          local_vehicle_footprint, autoware_utils_geometry::pose2transform(goal_pose));
        bool intersects_stop_or_parking = false;
        for (const auto & area : area_constraints.no_parking_areas) {
          if (boost::geometry::intersects(area, transformed)) intersects_stop_or_parking = true;
        }
        for (const auto & area : area_constraints.no_stopping_areas) {
          if (boost::geometry::intersects(area, transformed)) intersects_stop_or_parking = true;
        }
        if (intersects_stop_or_parking) break;
        continue;
      }
      GoalCandidate candidate{};
      candidate.goal_pose = goal_pose;
      candidate.lateral_offset = dy;
      candidate.id = goal_id++;
      candidate.distance_from_original_goal = seed.distance_from_original_goal;
      candidates.push_back(candidate);
    }
  }
  return candidates;
}
}  // namespace autoware::behavior_path_planner
