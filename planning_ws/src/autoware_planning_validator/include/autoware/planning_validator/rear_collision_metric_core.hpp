#ifndef AUTOWARE_PLANNING_VALIDATOR_REAR_COLLISION_METRIC_CORE_HPP_
#define AUTOWARE_PLANNING_VALIDATOR_REAR_COLLISION_METRIC_CORE_HPP_

namespace autoware::planning_validator
{
struct RearCollisionMetricInput
{
  double object_velocity{0.0};
  double relative_distance_with_delay_compensation{0.0};
  double object_moving_time{0.0};
  double ego_velocity{0.0};
  double distance_to_conflict_point{0.0};
  double reaction_time{0.0};
  double object_max_deceleration{0.0};
  double object_max_velocity{0.0};
  double ego_max_deceleration{0.0};
  double ego_min_velocity{0.0};
  double filter_min_velocity{0.0};
  double moving_time_threshold{0.0};
  double ttc_margin{0.0};
};

struct RearCollisionMetricResult
{
  bool safe{false};
  bool ignore{false};
  double value{0.0};  // RSS distance or time-to-collision, chosen by caller.
};

// Exact scalar equations from the upstream rear collision checker. This core
// has no lanelet or PCL assumptions; those remain mandatory in the future
// plugin that selects objects and determines distance_to_conflict_point.
RearCollisionMetricResult evaluate_rear_rss(const RearCollisionMetricInput & input);
RearCollisionMetricResult evaluate_rear_ttc(const RearCollisionMetricInput & input);
}  // namespace autoware::planning_validator

#endif
