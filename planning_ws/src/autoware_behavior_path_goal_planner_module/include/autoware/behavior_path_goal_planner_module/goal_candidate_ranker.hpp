#ifndef AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_RANKER_HPP_
#define AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_RANKER_HPP_

#include <autoware/behavior_path_goal_planner_module/goal_candidate.hpp>

#include <string>

namespace autoware::behavior_path_planner
{

// ROS1 ownership of GoalSearcher's source ranking branch. Unknown priorities
// intentionally preserve input order, matching the source update() behavior.
void rankGoalCandidates(
  GoalCandidates * candidates, const std::string & priority, double lateral_weight,
  bool prioritize_goals_before_objects);

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_RANKER_HPP_
