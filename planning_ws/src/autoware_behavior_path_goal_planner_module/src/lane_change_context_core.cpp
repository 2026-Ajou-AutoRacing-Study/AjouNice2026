#include <autoware/behavior_path_goal_planner_module/lane_change_context_core.hpp>

#include <autoware/route_handler/route_handler.hpp>

#include <algorithm>
#include <unordered_set>
#include <vector>

namespace autoware::behavior_path_planner
{
namespace
{
std::optional<lanelet::ConstLanelet> findLastLaneChangeCompletedLanelet(
  const autoware_internal_planning_msgs::PathWithLaneId & path,
  const lanelet::LaneletMapConstPtr & lanelet_map,
  const lanelet::routing::RoutingGraphConstPtr & routing_graph)
{
  if (!lanelet_map || !routing_graph) return std::nullopt;
  std::vector<lanelet::Id> reverse_path_lane_ids;
  for (auto point = path.points.rbegin(); point != path.points.rend(); ++point) {
    for (auto lane_id = point->lane_ids.rbegin(); lane_id != point->lane_ids.rend(); ++lane_id) {
      if (std::find(reverse_path_lane_ids.begin(), reverse_path_lane_ids.end(), *lane_id) == reverse_path_lane_ids.end()) {
        reverse_path_lane_ids.push_back(*lane_id);
      }
    }
  }
  if (reverse_path_lane_ids.size() < 2) return std::nullopt;
  for (size_t i = 0, j = 1; j < reverse_path_lane_ids.size(); ++i, ++j) {
    const auto lane_to = lanelet_map->laneletLayer.get(reverse_path_lane_ids.at(i));
    const auto lane_from = lanelet_map->laneletLayer.get(reverse_path_lane_ids.at(j));
    const auto previous = routing_graph->previous(lane_to);
    const auto is_following = std::any_of(previous.begin(), previous.end(), [&](const auto & lane) {
      return lane.id() == lane_from.id();
    });
    if (!is_following) return lane_to;
  }
  return std::nullopt;
}

std::vector<lanelet::Id> getLaneletSequenceAfter(
  lanelet::Id current_target_lane_id, lanelet::Id goal_lane_id,
  const lanelet::LaneletMapConstPtr & lanelet_map,
  const autoware::route_handler::RouteHandler & route_handler)
{
  std::vector<lanelet::Id> result;
  if (!lanelet_map) return result;
  auto current = lanelet_map->laneletLayer.get(current_target_lane_id);
  result.push_back(current.id());
  if (current_target_lane_id == goal_lane_id) return result;
  std::unordered_set<lanelet::Id> visited{current_target_lane_id};
  lanelet::ConstLanelet next;
  while (route_handler.getNextLaneletWithinRoute(current, &next)) {
    if (!visited.insert(next.id()).second) break;
    result.push_back(next.id());
    if (next.id() == goal_lane_id) return result;
    current = next;
  }
  return result;
}
}  // namespace

LaneChangeContextCore::State LaneChangeContextCore::getNextState(
  const autoware_internal_planning_msgs::PathWithLaneId & path, const lanelet::Id ego_lane_id,
  const autoware::route_handler::RouteHandler & route_handler, const ros::Time & now) const
{
  // Source policy: once a lane change is decided, later input degradation
  // cannot regress it to a non-decided state.
  if (isState<Completed>()) return state_;
  const auto lanelet_map = route_handler.getLaneletMapPtr();
  const auto routing_graph = route_handler.getRoutingGraphPtr();
  if (!lanelet_map || !routing_graph) return NotLaneChanging{};
  const auto complete_lane = findLastLaneChangeCompletedLanelet(path, lanelet_map, routing_graph);
  if (isState<Started>() || isState<Executing>()) {
    if (complete_lane) return isState<Started>() ? State{Executing(std::get<Started>(state_))} : state_;
    std::vector<lanelet::Id> lane_ids;
    for (const auto & point : path.points) {
      for (const auto lane_id : point.lane_ids) {
        if (std::find(lane_ids.begin(), lane_ids.end(), lane_id) == lane_ids.end()) lane_ids.push_back(lane_id);
      }
    }
    if (lane_ids.empty()) return Aborted{};
    const auto target = isState<Started>() ? std::get<Started>(state_).completeLane() : std::get<Executing>(state_).completeLane();
    const auto following_ids = getLaneletSequenceAfter(target, route_handler.getGoalLaneId(), lanelet_map, route_handler);
    const auto is_on_following = std::find(following_ids.begin(), following_ids.end(), ego_lane_id) != following_ids.end();
    const auto has_goal = std::find(lane_ids.begin(), lane_ids.end(), route_handler.getGoalLaneId()) != lane_ids.end();
    if (is_on_following && has_goal) {
      const auto exec = isState<Started>() ? Executing(std::get<Started>(state_)) : std::get<Executing>(state_);
      return Completed(exec);
    }
    return Aborted{};
  }
  return complete_lane ? State{Started(complete_lane->id(), now)} : State{NotLaneChanging{}};
}

bool LaneChangeContextCore::isNotConsistentTransition(const State & from, const State & to)
{
  if (isState<Started>(to) || isState<Aborted>(to)) return true;
  if (isState<Executing>(to)) {
    if (isState<Aborted>(from) || isState<NotLaneChanging>(from)) return true;
    const auto to_lane = std::get<Executing>(to).completeLane();
    const auto from_lane = isState<Started>(from) ? std::get<Started>(from).completeLane() : std::get<Executing>(from).completeLane();
    if (to_lane != from_lane) return true;
    const auto from_time = isState<Started>(from) ? std::get<Started>(from).startTime() : std::get<Executing>(from).startTime();
    return (std::get<Executing>(to).startTime() - from_time).toSec() > 0.0;
  }
  return isState<NotLaneChanging>(to) && !isState<NotLaneChanging>(from);
}

std::string LaneChangeContextCore::stateToString(const State & state)
{
  return std::visit([](const auto & value) {
    using T = std::decay_t<decltype(value)>;
    if constexpr (std::is_same_v<T, Started>) return std::string{"Started"};
    if constexpr (std::is_same_v<T, Executing>) return std::string{"Executing"};
    if constexpr (std::is_same_v<T, Aborted>) return std::string{"Aborted"};
    if constexpr (std::is_same_v<T, Completed>) return std::string{"Completed"};
    return std::string{"NotLaneChanging"};
  }, state);
}
}  // namespace autoware::behavior_path_planner
