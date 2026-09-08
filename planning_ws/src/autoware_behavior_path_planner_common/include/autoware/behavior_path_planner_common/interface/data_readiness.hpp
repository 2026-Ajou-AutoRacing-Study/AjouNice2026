#ifndef AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__INTERFACE__DATA_READINESS_HPP_
#define AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__INTERFACE__DATA_READINESS_HPP_

#include <optional>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{
// Mirrors BehaviorPathPlannerNode::DataReadyStatus in the frozen ROS2 source.
enum class DataReadyStatus {
  SUCCESS,
  NOT_RECEIVED,
  TIMEOUT,
};

// The ROS1 node adapter owns the messages and supplies only their reception state here.
// Timestamp values are ROS time in seconds and are intentionally node/message-type independent.
struct BppInputState {
  bool route_received{false};
  bool vector_map_received{false};
  bool scenario_received{false};
  bool acceleration_received{false};
  bool operation_mode_received{false};
  std::optional<double> perception_received_at_seconds;
  std::optional<double> odometry_received_at_seconds;
  std::optional<double> occupancy_grid_received_at_seconds;
  std::optional<double> traffic_signal_received_at_seconds;
};

struct BppInputReadinessReport {
  DataReadyStatus status{DataReadyStatus::NOT_RECEIVED};
  std::vector<std::string> not_received;
  std::vector<std::string> timed_out;

  // Upstream returns early only for NOT_RECEIVED; TIMEOUT is diagnostic-only.
  bool canRunPlanner() const { return status != DataReadyStatus::NOT_RECEIVED; }
};

// Preserves upstream BPP ingress semantics: mandatory inputs must be received once;
// cyclic inputs must be received once, while stale cyclic inputs produce TIMEOUT only.
BppInputReadinessReport evaluateBppInputReadiness(
  const BppInputState & state, double now_seconds, double cyclic_message_timeout_seconds,
  bool enable_traffic_signal_timeout);
}  // namespace autoware::behavior_path_planner

#endif
