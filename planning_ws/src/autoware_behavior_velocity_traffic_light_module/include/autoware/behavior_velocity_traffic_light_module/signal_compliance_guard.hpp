// Copyright 2026 AjouNice2026 contributors
// Licensed under the Apache License, Version 2.0.
#ifndef AUTOWARE_BEHAVIOR_VELOCITY_TRAFFIC_LIGHT_MODULE__SIGNAL_COMPLIANCE_GUARD_HPP_
#define AUTOWARE_BEHAVIOR_VELOCITY_TRAFFIC_LIGHT_MODULE__SIGNAL_COMPLIANCE_GUARD_HPP_

#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <stdexcept>

namespace autoware::behavior_velocity_planner::signal_compliance
{
// Authorization is route-direction-aware before it reaches this pure state machine.
enum class Observation { Missing, Stale, Unknown, Stop, AuthorizedGreen, Yellow };

inline const char * observationName(Observation value)
{
  switch(value) {
    case Observation::Missing: return "missing";
    case Observation::Stale: return "stale";
    case Observation::Unknown: return "unknown";
    case Observation::Stop: return "stop";
    case Observation::AuthorizedGreen: return "direction_authorized_green";
    case Observation::Yellow: return "yellow";
  }
  return "invalid";
}

class Guard
{
public:
  // Default three (configurable 1..3) distinct fresh, direction-authorized
  // source observations, not three planner callbacks or held/repeated stamps.
  static constexpr std::size_t requiredGreenObservations = 3;

  void setRequiredGreenObservations(std::size_t count)
  {
    if (count < 1 || count > 3) {
      throw std::invalid_argument("green confirmation count must be in 1..3");
    }
    if (required_green_observations_ != count) {
      reset();
      required_green_observations_ = count;
    }
  }

  bool observe(Observation observation, double source_sec, double now_sec, double timeout_sec)
  {
    const bool fresh = std::isfinite(source_sec) && source_sec > 0.0 &&
      std::isfinite(now_sec) && std::isfinite(timeout_sec) && timeout_sec > 0.0 &&
      now_sec >= source_sec && now_sec - source_sec <= timeout_sec;
    if (!fresh || observation == Observation::Missing || observation == Observation::Stale ||
        observation == Observation::Unknown) {
      reset();
      return true;
    }
    if ((last_now_ && now_sec < *last_now_) || (last_source_ && source_sec < *last_source_) ||
        (last_source_ && now_sec - *last_source_ > timeout_sec)) {
      reset();
    }
    if (observation == Observation::Yellow) {
      if (previous_ != Observation::Yellow) {
        yellow_from_authorized_green_ = previous_ == Observation::AuthorizedGreen &&
          green_authorized_ && last_source_ && source_sec > *last_source_ &&
          now_sec - *last_source_ <= timeout_sec;
        yellow_start_lower_bound_sec_ = yellow_from_authorized_green_ ? last_source_ : std::nullopt;
      }
      green_confirmations_ = 0;
      green_authorized_ = false;
    } else if (observation == Observation::AuthorizedGreen) {
      yellow_from_authorized_green_ = false;
      yellow_start_lower_bound_sec_.reset();
      if (previous_ != Observation::AuthorizedGreen) {
        green_confirmations_ = 0;
        green_authorized_ = false;
      }
      if ((!last_source_ || source_sec > *last_source_) &&
          green_confirmations_ < required_green_observations_) {
        ++green_confirmations_;
      }
      green_authorized_ = green_confirmations_ >= required_green_observations_;
    } else {
      yellow_from_authorized_green_ = false;
      yellow_start_lower_bound_sec_.reset();
      green_confirmations_ = 0;
      green_authorized_ = false;
    }
    previous_ = observation;
    last_source_ = source_sec;
    last_now_ = now_sec;
    // A yellow is initially STOP. Only the separate physical stopping-distance
    // test may permit an already-moving, freshly observed green->yellow transition.
    return observation != Observation::AuthorizedGreen || !green_authorized_;
  }

  bool yellowTransitionEligible() const { return yellow_from_authorized_green_; }
  std::size_t greenConfirmationCount() const { return green_confirmations_; }
  Observation lastObservation() const { return previous_; }
  std::optional<double> yellowStartLowerBoundSec() const { return yellow_start_lower_bound_sec_; }

  void reset()
  {
    previous_ = Observation::Missing;
    yellow_from_authorized_green_ = false;
    green_confirmations_ = 0;
    green_authorized_ = false;
    last_source_.reset();
    last_now_.reset();
    yellow_start_lower_bound_sec_.reset();
  }

private:
  Observation previous_{Observation::Missing};
  bool yellow_from_authorized_green_{false};
  std::size_t green_confirmations_{0};
  std::size_t required_green_observations_{requiredGreenObservations};
  bool green_authorized_{false};
  std::optional<double> last_source_;
  std::optional<double> last_now_;
  std::optional<double> yellow_start_lower_bound_sec_;
};

struct TimedYellowDecision
{
  bool allow{false};
  const char * reason{"disabled"};
  double remaining_sec{std::numeric_limits<double>::quiet_NaN()};
  double arrival_sec{std::numeric_limits<double>::infinity()};
};

inline TimedYellowDecision timedYellowDecision(
  bool enabled, bool transition_eligible, bool previous_stop, double velocity_mps,
  double acceleration_mps2, double slow_velocity_mps, double signed_front_line_distance_m,
  double now_sec, std::optional<double> yellow_start_lower_bound_sec,
  double min_yellow_sec, double time_margin_sec)
{
  TimedYellowDecision result;
  if (!enabled) { return result; }
  result.reason = "invalid_input";
  if (!std::isfinite(velocity_mps) || !std::isfinite(acceleration_mps2) ||
      !std::isfinite(slow_velocity_mps) || slow_velocity_mps <= 0.0 ||
      !std::isfinite(signed_front_line_distance_m) || signed_front_line_distance_m < 0.0 ||
      !std::isfinite(now_sec) || !std::isfinite(min_yellow_sec) || min_yellow_sec <= 0.0 ||
      !std::isfinite(time_margin_sec) || time_margin_sec < 0.0 || time_margin_sec >= min_yellow_sec) {
    return result;
  }
  result.reason = "no_observed_authorized_transition";
  if (!transition_eligible || !yellow_start_lower_bound_sec ||
      !std::isfinite(*yellow_start_lower_bound_sec) || *yellow_start_lower_bound_sec <= 0.0 ||
      *yellow_start_lower_bound_sec > now_sec) { return result; }
  result.remaining_sec = min_yellow_sec - (now_sec - *yellow_start_lower_bound_sec);
  result.reason = "already_stopped_or_slow";
  if (previous_stop || velocity_mps < slow_velocity_mps) { return result; }
  result.reason = "assumed_yellow_budget_exhausted";
  if (result.remaining_sec <= time_margin_sec) { return result; }
  // Do not assume future positive acceleration. Retain current negative
  // acceleration for this estimate; this is a model, not a physical bound.
  const double a = std::fmin(0.0, acceleration_mps2);
  if (a < -1e-6) {
    const double discriminant = velocity_mps*velocity_mps + 2.0*a*signed_front_line_distance_m;
    result.reason = "current_braking_stops_before_crossing";
    if (!std::isfinite(discriminant) || discriminant <= 0.0) { return result; }
    result.arrival_sec = 2.0*signed_front_line_distance_m /
      (velocity_mps + std::sqrt(discriminant));
  } else {
    result.arrival_sec = signed_front_line_distance_m / velocity_mps;
  }
  result.allow = std::isfinite(result.arrival_sec) &&
    result.arrival_sec + time_margin_sec <= result.remaining_sec;
  result.reason = result.allow ? "observed_transition_time_fits" : "insufficient_arrival_margin";
  return result;
}

inline std::optional<double> candidateStoppingDistance(
  double velocity_mps, double acceleration_mps2, double max_stop_acceleration_mps2,
  double max_stop_jerk_mps3, double delay_response_sec, double legacy_distance_m)
{
  if (!std::isfinite(velocity_mps) || !std::isfinite(acceleration_mps2) ||
      !std::isfinite(max_stop_acceleration_mps2) || !std::isfinite(max_stop_jerk_mps3) ||
      !std::isfinite(delay_response_sec) || !std::isfinite(legacy_distance_m) ||
      max_stop_acceleration_mps2 >= 0.0 || max_stop_jerk_mps3 >= 0.0 ||
      delay_response_sec < 0.0 || legacy_distance_m < 0.0) {
    return std::nullopt;
  }
  if (velocity_mps <= 0.0) {
    return 0.0;
  }
  const double velocity_at_acceleration_limit = velocity_mps +
    (max_stop_acceleration_mps2 * max_stop_acceleration_mps2 -
     acceleration_mps2 * acceleration_mps2) / (2.0 * max_stop_jerk_mps3);
  if (!std::isfinite(velocity_at_acceleration_limit)) {
    return std::nullopt;
  }
  if (velocity_at_acceleration_limit > 0.0) {
    // Bounded traffic-light-only correction: leave the common library's
    // trapezoidal branch byte-for-byte numerically unchanged.
    return legacy_distance_m;
  }

  // Triangular braking stops before reaching the acceleration limit. The
  // positive root of v + a0*t + j*t^2/2 = 0 must use current acceleration a0,
  // NOT the unused maximum braking acceleration. Rationalize for a0<0 to
  // avoid subtracting two nearly equal values at very low forward velocity.
  const double discriminant = acceleration_mps2 * acceleration_mps2 -
    2.0 * max_stop_jerk_mps3 * velocity_mps;
  if (!std::isfinite(discriminant) || discriminant < 0.0) {
    return std::nullopt;
  }
  const double root = std::sqrt(discriminant);
  const double stop_time = acceleration_mps2 < 0.0 ?
    2.0 * velocity_mps / (root - acceleration_mps2) :
    -(acceleration_mps2 + root) / max_stop_jerk_mps3;
  if (!std::isfinite(stop_time) || stop_time < 0.0) {
    return std::nullopt;
  }
  // Retain the original constant-velocity response-delay approximation.
  // This is not a guarantee of the measured vehicle's physical braking range.
  const double distance = velocity_mps * delay_response_sec + velocity_mps * stop_time +
    acceleration_mps2 * stop_time * stop_time / 2.0 +
    max_stop_jerk_mps3 * stop_time * stop_time * stop_time / 6.0;
  return std::isfinite(distance) ? std::optional<double>(std::fmax(0.0, distance)) : std::nullopt;
}

inline bool yellowPassthroughAllowed(
  bool enabled, bool transition_eligible, bool previous_stop, double velocity_mps,
  double slow_velocity_mps, double stopping_distance_m, double signed_front_line_distance_m)
{
  // No yellow period or presumed remaining-yellow time is used here. This cannot
  // guarantee a compliant crossing if red arrives before physical stopping is possible.
  return enabled && transition_eligible && !previous_stop &&
    std::isfinite(velocity_mps) && velocity_mps > 0.0 &&
    std::isfinite(slow_velocity_mps) && slow_velocity_mps > 0.0 &&
    velocity_mps >= slow_velocity_mps && std::isfinite(stopping_distance_m) &&
    stopping_distance_m >= 0.0 && std::isfinite(signed_front_line_distance_m) &&
    stopping_distance_m >= signed_front_line_distance_m;
}

inline std::optional<double> frontAxleDistanceFromBumperStop(
  double bumper_stop_distance_m, double stop_margin_m, double front_bumper_offset_m,
  double explicit_front_axle_offset_m)
{
  if (!std::isfinite(bumper_stop_distance_m) || !std::isfinite(stop_margin_m) ||
      !std::isfinite(front_bumper_offset_m) || !std::isfinite(explicit_front_axle_offset_m) ||
      stop_margin_m < 0.0 || explicit_front_axle_offset_m < 0.0 ||
      front_bumper_offset_m < explicit_front_axle_offset_m) {
    return std::nullopt;
  }
  // Local path-arc approximation, NOT a simulator/front-wheel GT transform.
  // The caller must explicitly qualify base_link and front-axle conventions.
  return bumper_stop_distance_m + stop_margin_m + front_bumper_offset_m -
         explicit_front_axle_offset_m;
}
}  // namespace autoware::behavior_velocity_planner::signal_compliance
#endif
