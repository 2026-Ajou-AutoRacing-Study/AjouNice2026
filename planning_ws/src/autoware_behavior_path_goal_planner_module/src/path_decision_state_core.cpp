#include <autoware/behavior_path_goal_planner_module/path_decision_state_core.hpp>

namespace autoware::behavior_path_planner
{
void PathDecisionStateCore::transition(
  const Input & input, const ros::Time & now, const Parameters & parameters)
{
  auto next = state_;
  if (input.current_path_safe) {
    if (!next.safe_start_time) {
      next.safe_start_time = now;
      next.is_stable_safe = false;
    } else {
      next.is_stable_safe = (now - *next.safe_start_time).toSec() > parameters.keep_unsafe_time;
    }
  } else {
    next.safe_start_time = std::nullopt;
    next.is_stable_safe = false;
  }

  if (next.kind == DecisionKind::Decided) { state_ = next; return; }
  if (input.lane_change_status_changed || input.upstream_has_stopline || !input.has_pull_over_path) {
    next.kind = DecisionKind::NotDecided;
    state_ = next;
    return;
  }
  if (!next.is_stable_safe || !input.goal_safe || input.parking_path_collision) {
    next.kind = DecisionKind::NotDecided;
    next.deciding_start_time = std::nullopt;
    state_ = next;
    return;
  }
  if (state_.kind == DecisionKind::Deciding) {
    if (!state_.deciding_start_time ||
        (now - *state_.deciding_start_time).toSec() > parameters.check_collision_duration) {
      next.kind = DecisionKind::Decided;
      next.deciding_start_time = std::nullopt;
    }
    state_ = next;
    return;
  }
  if (input.distance_to_parking_start > parameters.decide_path_distance) {
    next.kind = DecisionKind::NotDecided;
    state_ = next;
    return;
  }
  next.kind = DecisionKind::Deciding;
  next.deciding_start_time = now;
  state_ = next;
}
}  // namespace autoware::behavior_path_planner
