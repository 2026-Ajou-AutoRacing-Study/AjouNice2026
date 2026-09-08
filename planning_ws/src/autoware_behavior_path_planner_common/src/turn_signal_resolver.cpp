#include <autoware/behavior_path_planner_common/turn_signal_resolver.hpp>

#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <algorithm>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{
namespace
{
struct SignalCandidate
{
  TurnSignalInfo signal_info;
  double desired_start_distance;
  double desired_end_distance;
  double required_start_distance;
  double required_end_distance;
  std::string signal_type;

  bool isValid() const
  {
    const auto command = signal_info.turn_signal.command;
    return command != autoware_vehicle_msgs::TurnIndicatorsCommand::NO_COMMAND &&
           command != autoware_vehicle_msgs::TurnIndicatorsCommand::DISABLE &&
           desired_start_distance <= 0.0 && desired_end_distance >= 0.0;
  }
};

bool usePriorTurnSignal(
  const double prior_required_start, const double prior_required_end,
  const double subsequent_required_start, const double subsequent_required_end)
{
  const bool before_prior_required = prior_required_start > 0.0;
  const bool before_subsequent_required = subsequent_required_start > 0.0;
  const bool inside_prior_required = prior_required_start < 0.0 && 0.0 <= prior_required_end;
  if (prior_required_start < subsequent_required_start) {
    if (subsequent_required_end < prior_required_end) return true;
    if (before_prior_required || inside_prior_required) return true;
    if (before_subsequent_required) return true;
    return false;
  }
  if (prior_required_end < subsequent_required_end) {
    return before_prior_required || inside_prior_required;
  }
  if (before_prior_required) return false;
  return true;
}

int priority(const std::string & type)
{
  if (type == "intersection") return 0;
  if (type == "roundabout") return 1;
  return 2;
}
}  // namespace

autoware_vehicle_msgs::TurnIndicatorsCommand TurnSignalResolver::resolve(
  const autoware_internal_planning_msgs::PathWithLaneId & path,
  const geometry_msgs::Pose & current_pose, const size_t current_segment_index,
  const TurnSignalInfo & intersection_signal_info,
  const TurnSignalInfo & roundabout_signal_info,
  const TurnSignalInfo & behavior_signal_info,
  const double nearest_dist_threshold, const double nearest_yaw_threshold) const
{
  autoware_vehicle_msgs::TurnIndicatorsCommand disabled;
  disabled.command = autoware_vehicle_msgs::TurnIndicatorsCommand::DISABLE;
  if (path.points.empty() || current_segment_index >= path.points.size()) return disabled;

  const auto distance = [&](const geometry_msgs::Pose & pose) {
    const size_t segment_index = autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, pose, nearest_dist_threshold, nearest_yaw_threshold);
    return autoware::motion_utils::calcSignedArcLength(
      path.points, current_pose.position, current_segment_index, pose.position, segment_index);
  };
  const auto candidate = [&](const TurnSignalInfo & signal, const std::string & type) {
    return SignalCandidate{
      signal, distance(signal.desired_start_point) - base_link2front_,
      distance(signal.desired_end_point), distance(signal.required_start_point) - base_link2front_,
      distance(signal.required_end_point), type};
  };

  std::vector<SignalCandidate> candidates;
  candidates.reserve(3);
  candidates.push_back(candidate(intersection_signal_info, "intersection"));
  candidates.push_back(candidate(roundabout_signal_info, "roundabout"));
  candidates.push_back(candidate(behavior_signal_info, "behavior"));
  candidates.erase(std::remove_if(candidates.begin(), candidates.end(), [](const auto & value) {
    return !value.isValid();
  }), candidates.end());
  if (candidates.empty()) return disabled;

  std::sort(candidates.begin(), candidates.end(), [](const auto & left, const auto & right) {
    if (left.desired_start_distance != right.desired_start_distance) {
      return left.desired_start_distance < right.desired_start_distance;
    }
    return priority(left.signal_type) < priority(right.signal_type);
  });
  SignalCandidate selected = candidates.front();
  for (size_t index = 1; index < candidates.size(); ++index) {
    const auto & next = candidates.at(index);
    if (!usePriorTurnSignal(
          selected.required_start_distance, selected.required_end_distance,
          next.required_start_distance, next.required_end_distance)) {
      selected = next;
    }
  }
  return selected.signal_info.turn_signal;
}

}  // namespace autoware::behavior_path_planner
