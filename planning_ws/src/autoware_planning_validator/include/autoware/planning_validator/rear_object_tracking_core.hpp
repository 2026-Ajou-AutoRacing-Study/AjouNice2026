#ifndef AUTOWARE_PLANNING_VALIDATOR_REAR_OBJECT_TRACKING_CORE_HPP_
#define AUTOWARE_PLANNING_VALIDATOR_REAR_OBJECT_TRACKING_CORE_HPP_

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace autoware::planning_validator
{
// ROS1-neutral representation of the fields updated by upstream
// RearCollisionChecker::fill_velocity. Time values are seconds on the caller's
// ROS clock; the node adapter must never substitute wall time for ROS time.
struct RearObjectTrackingState
{
  int64_t lane_id{0};
  double last_update_time{0.0};
  double last_stop_time{0.0};
  double tracking_duration{0.0};
  double relative_distance{0.0};
  double relative_distance_with_delay_compensation{0.0};
  double velocity{0.0};
  double moving_time{0.0};
  std::string detail;
};

struct RearObjectTrackingConfig
{
  double latency{0.0};
  double observation_time{0.0};
  double max_acceleration{0.0};
  double min_velocity{0.0};
};

// Stores lane-keyed observations exactly as the upstream checker does. For a
// newly encountered lane, previous_lane_ids must be supplied in the exact
// RouteHandler order; the first known predecessor is adopted. A missing
// predecessor deliberately leaves the input as an untracked first observation.
class RearObjectTrackingCore
{
public:
  explicit RearObjectTrackingCore(RearObjectTrackingConfig config);

  void update(
    RearObjectTrackingState & state, double ego_velocity, double ros_now,
    const std::vector<int64_t> & previous_lane_ids);

  const std::map<int64_t, RearObjectTrackingState> & history() const { return history_; }

private:
  void fill_from_previous(
    RearObjectTrackingState & state, const RearObjectTrackingState & previous,
    double ego_velocity, double ros_now);
  void update_history(const RearObjectTrackingState & state);

  RearObjectTrackingConfig config_;
  std::map<int64_t, RearObjectTrackingState> history_;
};
}  // namespace autoware::planning_validator

#endif
