#ifndef AUTOWARE_PLANNING_VALIDATOR_REAR_COLLISION_DECISION_CORE_HPP_
#define AUTOWARE_PLANNING_VALIDATOR_REAR_COLLISION_DECISION_CORE_HPP_

#include <vector>

namespace autoware::planning_validator
{
struct RearRangeInput
{
  double range_buffer{0.0};
  double ego_max_longitudinal_offset{0.0};
  double ego_min_longitudinal_offset{0.0};
  double ego_velocity{0.0};
  double ego_max_deceleration{0.0};
  double ego_min_velocity{0.0};
  bool blind_spot_check_front{false};
  double participant_reaction_time{0.0};
  double participant_max_deceleration{0.0};
  double participant_max_velocity{0.0};
  double distance_to_conflict_point{0.0};
  double ttc_margin{0.0};
};

struct RearDetectionRange
{
  double forward{0.0};
  double backward{0.0};
};

RearDetectionRange rear_range_for_rss(const RearRangeInput & input);
RearDetectionRange rear_range_for_ttc(const RearRangeInput & input);

struct RearSafetyObject
{
  double tracking_duration{0.0};
  bool ignore{false};
  bool safe{true};
};

struct RearObjectSafetyInput
{
  double relative_distance_with_delay_compensation{0.0};
  double velocity{0.0};
  double moving_time{0.0};
  double ego_velocity{0.0};
  double ego_min_velocity{0.0};
  double ego_max_deceleration{0.0};
  double participant_max_deceleration{0.0};
  double participant_max_velocity{0.0};
  double participant_reaction_time{0.0};
  double distance_to_conflict_point{0.0};
  double min_velocity{0.0};
  double moving_time_threshold{0.0};
  double ttc_margin{0.0};
};

// Exact source fill_time_to_collision/fill_rss_distance reduction used after
// velocity tracking. The return contains the final mature-object safety flags.
RearSafetyObject evaluate_rear_ttc_safety(const RearObjectSafetyInput & input);
RearSafetyObject evaluate_rear_rss_safety(const RearObjectSafetyInput & input);

bool rear_objects_are_safe(
  const std::vector<RearSafetyObject> & objects, double observation_time);

struct RearSafetyDebounceState
{
  double last_safe_time{0.0};
  double last_unsafe_time{0.0};
};

// Exact final on/off-buffer branch from RearCollisionChecker::is_safe. The
// caller owns event timestamps and must pass the ROS clock value used for the
// pointcloud update, not wall clock time.
bool apply_rear_safety_debounce(
  bool objects_safe, double ros_now, double on_time_buffer, double off_time_buffer,
  RearSafetyDebounceState & state);
}  // namespace autoware::planning_validator

#endif
