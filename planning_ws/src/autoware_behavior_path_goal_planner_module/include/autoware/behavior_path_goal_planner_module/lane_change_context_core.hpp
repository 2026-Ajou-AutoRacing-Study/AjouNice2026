#pragma once

#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <ros/time.h>

#include <optional>
#include <string>
#include <type_traits>
#include <variant>

namespace autoware::route_handler
{
class RouteHandler;
}

namespace autoware::behavior_path_planner
{
// ROS1 ownership of the source Goal Planner lane-change transition state.
// This remains separate from pull-over selection and only consumes a ready
// RouteHandler plus the upstream PathWithLaneId result.
class LaneChangeContextCore
{
public:
  class Started
  {
  public:
    Started(lanelet::Id complete_lane, const ros::Time & start_time)
    : complete_lane_(complete_lane), start_time_(start_time) {}
    lanelet::Id completeLane() const { return complete_lane_; }
    ros::Time startTime() const { return start_time_; }
  private:
    lanelet::Id complete_lane_{};
    ros::Time start_time_{};
  };
  class Executing
  {
  public:
    explicit Executing(const Started & from)
    : complete_lane_(from.completeLane()), start_time_(from.startTime()) {}
    lanelet::Id completeLane() const { return complete_lane_; }
    ros::Time startTime() const { return start_time_; }
  private:
    lanelet::Id complete_lane_{};
    ros::Time start_time_{};
  };
  struct Aborted {};
  struct Completed { explicit Completed(const Executing &) {} };
  struct NotLaneChanging {};
  using State = std::variant<Started, Executing, Aborted, Completed, NotLaneChanging>;

  State getNextState(
    const autoware_internal_planning_msgs::PathWithLaneId & path, lanelet::Id ego_lane_id,
    const autoware::route_handler::RouteHandler & route_handler, const ros::Time & now) const;
  State currentState() const { return state_; }

  template <typename T> bool isState() const { return std::holds_alternative<T>(state_); }
  template <typename T> static bool isState(const State & state) { return std::holds_alternative<T>(state); }

  void setState(const State & state)
  {
    is_in_consistent_transition_ = !isNotConsistentTransition(state_, state);
    state_ = state;
  }
  static bool isNotConsistentTransition(const State & from, const State & to);
  bool isInConsistentTransition() const { return is_in_consistent_transition_; }
  static std::string stateToString(const State & state);

private:
  State state_{NotLaneChanging{}};
  bool is_in_consistent_transition_{};
};
}  // namespace autoware::behavior_path_planner
