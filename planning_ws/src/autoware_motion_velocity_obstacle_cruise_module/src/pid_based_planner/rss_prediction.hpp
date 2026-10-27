// Copyright 2026 AjouNice contributors
// Licensed under the Apache License, Version 2.0.
#ifndef PID_BASED_PLANNER__RSS_PREDICTION_HPP_
#define PID_BASED_PLANNER__RSS_PREDICTION_HPP_

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace autoware::motion_velocity_planner
{
struct RSSPredictionParameters
{
  bool enabled{false};
  double start_relative_speed_kph{30.0};
  double full_relative_speed_kph{80.0};
  double max_time{1.0};

  void validate(const double fixed_time) const
  {
    if (!std::isfinite(fixed_time) || fixed_time < 0.0 ||
        !std::isfinite(start_relative_speed_kph) || start_relative_speed_kph < 0.0 ||
        !std::isfinite(full_relative_speed_kph) ||
        full_relative_speed_kph <= start_relative_speed_kph ||
        !std::isfinite(max_time) || max_time < 0.0) {
      throw std::invalid_argument("obstacle_cruise RSS prediction parameters are invalid");
    }
  }
};

struct RSSPrediction
{
  double time{0.0};
  double projected_distance{0.0};
  double correction{0.0};
};

// Prediction changes the distance ERROR, never the physical distance used as
// the PID denominator, TTC distance, or published obstacle distance.
inline RSSPrediction evaluate_rss_prediction(
  const double distance, const double closing_velocity, const double fixed_time,
  const RSSPredictionParameters & parameters)
{
  RSSPrediction result{0.0, distance, 0.0};
  if (closing_velocity <= 0.0) return result;
  result.time = parameters.enabled
    ? parameters.max_time * std::clamp(
        (closing_velocity * 3.6 - parameters.start_relative_speed_kph) /
          (parameters.full_relative_speed_kph - parameters.start_relative_speed_kph),
        0.0, 1.0)
    : fixed_time;
  const double approach = closing_velocity * result.time;
  result.projected_distance = distance - approach;
  if (approach <= 0.0 || result.projected_distance <= 0.0) return result;

  // Fade over one approach distance: full effect for d >= 2*approach,
  // zero effect for d <= approach, with zero slope at either end. This is
  // stateless so candidate order and UUID changes cannot share prediction state.
  const double ratio = std::clamp(result.projected_distance / approach, 0.0, 1.0);
  const double weight = ratio * ratio * (3.0 - 2.0 * ratio);
  result.correction = approach * weight;
  return result;
}

inline double normalize_cruise_distance_error(const double error, const double distance)
{
  // A zero/negative signed collision-point distance must retain a braking sign.
  constexpr double minimum_distance = 0.1;  // numerical denominator floor [m], not an RSS gap
  return error / std::max(distance, minimum_distance);
}
}  // namespace autoware::motion_velocity_planner
#endif  // PID_BASED_PLANNER__RSS_PREDICTION_HPP_
