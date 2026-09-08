#include "autoware/planning_validator/rear_collision_metric_core.hpp"

#include <algorithm>
#include <cmath>

namespace autoware::planning_validator
{
namespace
{
bool ignored(const RearCollisionMetricInput & input)
{
  return input.object_moving_time < input.moving_time_threshold ||
         input.object_velocity > input.object_max_velocity;
}
}  // namespace

RearCollisionMetricResult evaluate_rear_rss(const RearCollisionMetricInput & input)
{
  const double object_stop_distance =
    input.reaction_time * input.object_velocity +
    0.5 * std::pow(input.object_velocity, 2.0) / std::abs(input.object_max_deceleration);
  const double ego_stop_distance =
    0.5 * std::pow(input.ego_velocity, 2.0) / std::abs(input.ego_max_deceleration);
  const double rss_distance = object_stop_distance - ego_stop_distance;
  return {rss_distance < input.relative_distance_with_delay_compensation, ignored(input), rss_distance};
}

RearCollisionMetricResult evaluate_rear_ttc(const RearCollisionMetricInput & input)
{
  const double ego_time = input.distance_to_conflict_point /
    std::max(input.ego_velocity, input.ego_min_velocity);
  const double object_time = (input.distance_to_conflict_point +
    input.relative_distance_with_delay_compensation) /
    std::max(input.object_velocity, input.filter_min_velocity);
  const double ttc = std::abs(ego_time - object_time);
  return {ttc > input.ttc_margin, ignored(input), ttc};
}
}  // namespace autoware::planning_validator
