#include "autoware/planning_validator/rear_object_tracking_core.hpp"

#include <cmath>

namespace autoware::planning_validator
{
RearObjectTrackingCore::RearObjectTrackingCore(RearObjectTrackingConfig config) : config_(config) {}

void RearObjectTrackingCore::update_history(const RearObjectTrackingState & state)
{
  history_[state.lane_id] = state;
}

void RearObjectTrackingCore::fill_from_previous(
  RearObjectTrackingState & state, const RearObjectTrackingState & previous,
  const double ego_velocity, const double ros_now)
{
  const double dt = state.last_update_time - previous.last_update_time;
  if (dt < 1e-6) {
    state.velocity = previous.velocity;
    state.tracking_duration = previous.tracking_duration;
    state.relative_distance_with_delay_compensation =
      state.relative_distance - state.velocity * config_.latency;
  } else {
    const double raw_velocity =
      (previous.relative_distance - state.relative_distance) / dt + ego_velocity;
    const bool is_reliable = previous.tracking_duration > config_.observation_time;
    if (
      is_reliable &&
      std::abs(raw_velocity - previous.velocity) / dt > config_.max_acceleration) {
      state.velocity = previous.velocity;
      state.tracking_duration = previous.tracking_duration;
      state.detail = "the estimated velocity may be an outlier.";
    } else {
      // autoware::signal_processing::lowpassFilter(raw, prev, 0.5).
      state.velocity = 0.5 * previous.velocity + 0.5 * raw_velocity;
      state.tracking_duration = previous.tracking_duration + dt;
    }
    state.relative_distance_with_delay_compensation =
      state.relative_distance - state.velocity * config_.latency;
  }

  if (state.velocity > config_.min_velocity) {
    state.moving_time = previous.moving_time + dt;
    state.last_stop_time = previous.last_stop_time;
  } else {
    state.moving_time = 0.0;
    state.last_stop_time = ros_now;
  }
}

void RearObjectTrackingCore::update(
  RearObjectTrackingState & state, const double ego_velocity, const double ros_now,
  const std::vector<int64_t> & previous_lane_ids)
{
  const auto current = history_.find(state.lane_id);
  if (current != history_.end()) {
    fill_from_previous(state, current->second, ego_velocity, ros_now);
    update_history(state);
    return;
  }

  for (const auto lane_id : previous_lane_ids) {
    const auto previous = history_.find(lane_id);
    if (previous == history_.end()) continue;
    fill_from_previous(state, previous->second, ego_velocity, ros_now);
    update_history(state);
    return;
  }

  update_history(state);
}
}  // namespace autoware::planning_validator
