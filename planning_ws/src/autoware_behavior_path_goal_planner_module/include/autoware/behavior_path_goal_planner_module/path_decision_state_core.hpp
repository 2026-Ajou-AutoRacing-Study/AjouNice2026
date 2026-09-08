#pragma once

#include <ros/time.h>

#include <optional>

namespace autoware::behavior_path_planner
{
// Source Goal Planner decision-state transition with all geometry/object
// predicates made explicit at the ROS1 boundary. Callers may not replace an
// unavailable predicate with a synthetic safe value.
class PathDecisionStateCore
{
public:
  enum class DecisionKind { NotDecided, Deciding, Decided };
  struct Parameters
  {
    double keep_unsafe_time{};
    double check_collision_duration{};
    double decide_path_distance{};
  };
  struct Input
  {
    bool has_pull_over_path{};
    bool upstream_has_stopline{};
    bool current_path_safe{};
    bool lane_change_status_changed{};
    bool goal_safe{};
    bool parking_path_collision{};
    double distance_to_parking_start{};
  };
  struct State
  {
    DecisionKind kind{DecisionKind::NotDecided};
    std::optional<ros::Time> deciding_start_time{};
    bool is_stable_safe{};
    std::optional<ros::Time> safe_start_time{};
  };

  const State & state() const { return state_; }
  void transition(const Input & input, const ros::Time & now, const Parameters & parameters);

private:
  State state_{};
};
}  // namespace autoware::behavior_path_planner
