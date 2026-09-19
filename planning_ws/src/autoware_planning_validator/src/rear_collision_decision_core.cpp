#include "autoware/planning_validator/rear_collision_decision_core.hpp"

#include <algorithm>
#include <cmath>

namespace autoware::planning_validator
{
RearDetectionRange rear_range_for_rss(const RearRangeInput & input)
{
  const double object_stop_distance =
    input.participant_reaction_time * input.participant_max_velocity +
    0.5 * std::pow(input.participant_max_velocity, 2.0) /
      std::abs(input.participant_max_deceleration);
  const double ego_stop_distance =
    0.5 * std::pow(input.ego_velocity, 2.0) / std::abs(input.ego_max_deceleration);
  return {
    input.range_buffer + std::max(
      input.ego_max_longitudinal_offset,
      input.blind_spot_check_front ? ego_stop_distance : 0.0),
    input.range_buffer - input.ego_min_longitudinal_offset +
      std::max(0.0, object_stop_distance - ego_stop_distance)};
}

RearDetectionRange rear_range_for_ttc(const RearRangeInput & input)
{
  const double ego_time = input.distance_to_conflict_point /
    std::max(input.ego_velocity, input.ego_min_velocity);
  return {
    input.range_buffer + input.ego_max_longitudinal_offset,
    input.range_buffer +
      (ego_time + input.ttc_margin) * input.participant_max_velocity -
      input.distance_to_conflict_point};
}

RearSafetyObject evaluate_rear_ttc_safety(const RearObjectSafetyInput & input)
{
  const auto ego_time = input.distance_to_conflict_point /
    std::max(input.ego_velocity, input.ego_min_velocity);
  const auto object_time =
    (input.distance_to_conflict_point + input.relative_distance_with_delay_compensation) /
    std::max(input.velocity, input.min_velocity);
  return {
    0.0,
    input.moving_time < input.moving_time_threshold || input.velocity > input.participant_max_velocity,
    std::abs(ego_time - object_time) > input.ttc_margin};
}

RearSafetyObject evaluate_rear_rss_safety(const RearObjectSafetyInput & input)
{
  const auto object_stop_distance =
    input.participant_reaction_time * input.velocity +
    0.5 * std::pow(input.velocity, 2.0) / std::abs(input.participant_max_deceleration);
  const auto ego_stop_distance =
    0.5 * std::pow(input.ego_velocity, 2.0) / std::abs(input.ego_max_deceleration);
  return {
    0.0,
    input.moving_time < input.moving_time_threshold || input.velocity > input.participant_max_velocity,
    object_stop_distance - ego_stop_distance < input.relative_distance_with_delay_compensation};
}

bool rear_objects_are_safe(
  const std::vector<RearSafetyObject> & objects, const double observation_time)
{
  for (const auto & object : objects) {
    if (object.tracking_duration < observation_time) continue;
    if (object.ignore) continue;
    if (object.safe) continue;
    return false;
  }
  return true;
}

bool apply_rear_safety_debounce(
  const bool objects_safe, const double ros_now, const double on_time_buffer,
  const double off_time_buffer, RearSafetyDebounceState & state)
{
  if (objects_safe) {
    if (ros_now - state.last_unsafe_time > off_time_buffer) state.last_safe_time = ros_now;
    return true;
  }
  if (ros_now - state.last_safe_time < on_time_buffer) return true;
  state.last_unsafe_time = ros_now;
  return false;
}
}  // namespace autoware::planning_validator
