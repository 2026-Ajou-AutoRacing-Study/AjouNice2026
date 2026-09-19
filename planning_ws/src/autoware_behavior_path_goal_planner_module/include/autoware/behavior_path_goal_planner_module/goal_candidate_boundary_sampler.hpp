#ifndef AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_BOUNDARY_SAMPLER_HPP_
#define AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_BOUNDARY_SAMPLER_HPP_

#include <geometry_msgs/Pose.h>
#include <lanelet2_core/primitives/LineString.h>

#include <vector>

namespace autoware::behavior_path_planner
{

// Source GoalSearcher cropBoundary projection. Invalid sampling configuration
// returns no samples rather than spinning indefinitely.
std::vector<geometry_msgs::Pose> sampleGoalCandidateBoundary(
  const lanelet::ConstLineString3d & boundary, const geometry_msgs::Pose & reference_pose,
  double backward_length, double forward_length, double interval);

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_BOUNDARY_SAMPLER_HPP_
