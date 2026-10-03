#ifndef AVOIDANCE_LANE_CHANGE_REQUEST_CONDITION_HPP_
#define AVOIDANCE_LANE_CHANGE_REQUEST_CONDITION_HPP_
#include <optional>
#include <string>
namespace autoware::behavior_path_planner::avoidance_request
{
// isLaneChangeRequired returns an error, not a boolean request. An engaged
// optional (including an empty error string) must never authorize execution.
inline bool allowed(const std::optional<std::string> & error, bool special, bool valid)
{
  return !error.has_value() && special && valid;
}
}
#endif
