#include <autoware/behavior_path_goal_planner_module/goal_candidate_ranker.hpp>

#include <algorithm>
#include <cmath>

namespace autoware::behavior_path_planner
{
namespace
{
struct ByLongitudinalDistance
{
  bool prioritize_objects{false};

  bool operator()(const GoalCandidate & left, const GoalCandidate & right) const noexcept
  {
    if (prioritize_objects && left.num_objects_to_avoid != right.num_objects_to_avoid) {
      return left.num_objects_to_avoid < right.num_objects_to_avoid;
    }
    constexpr double epsilon = 0.01;
    const double difference = left.distance_from_original_goal - right.distance_from_original_goal;
    return std::abs(difference) < epsilon
      ? left.lateral_offset < right.lateral_offset
      : left.distance_from_original_goal < right.distance_from_original_goal;
  }
};

struct ByWeightedDistance
{
  double lateral_weight{0.0};
  bool prioritize_objects{false};

  bool operator()(const GoalCandidate & left, const GoalCandidate & right) const noexcept
  {
    if (prioritize_objects && left.num_objects_to_avoid != right.num_objects_to_avoid) {
      return left.num_objects_to_avoid < right.num_objects_to_avoid;
    }
    return left.distance_from_original_goal + lateral_weight * left.lateral_offset <
           right.distance_from_original_goal + lateral_weight * right.lateral_offset;
  }
};
}  // namespace

void rankGoalCandidates(
  GoalCandidates * candidates, const std::string & priority, const double lateral_weight,
  const bool prioritize_goals_before_objects)
{
  if (!candidates) return;
  if (priority == "minimum_weighted_distance") {
    std::sort(
      candidates->begin(), candidates->end(),
      ByWeightedDistance{lateral_weight, prioritize_goals_before_objects});
  } else if (priority == "minimum_longitudinal_distance") {
    std::sort(
      candidates->begin(), candidates->end(),
      ByLongitudinalDistance{prioritize_goals_before_objects});
  }
}

}  // namespace autoware::behavior_path_planner
