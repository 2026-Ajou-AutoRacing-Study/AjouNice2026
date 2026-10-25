// Copyright 2026 AjouNice2026 contributors
// Licensed under the Apache License, Version 2.0.
#ifndef AUTOWARE_BEHAVIOR_VELOCITY_TRAFFIC_LIGHT_MODULE__SIGNAL_COMPLIANCE_GUARD_HPP_
#define AUTOWARE_BEHAVIOR_VELOCITY_TRAFFIC_LIGHT_MODULE__SIGNAL_COMPLIANCE_GUARD_HPP_

#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>

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
  // Candidate-only fixed contract: three distinct fresh, direction-authorized
  // source observations, not three planner callbacks or held/repeated stamps.
  static constexpr std::size_t requiredGreenObservations = 3;

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
      }
      green_confirmations_ = 0;
      green_authorized_ = false;
    } else if (observation == Observation::AuthorizedGreen) {
      yellow_from_authorized_green_ = false;
      if (previous_ != Observation::AuthorizedGreen) {
        green_confirmations_ = 0;
        green_authorized_ = false;
      }
      if ((!last_source_ || source_sec > *last_source_) &&
          green_confirmations_ < requiredGreenObservations) {
        ++green_confirmations_;
      }
      green_authorized_ = green_confirmations_ >= requiredGreenObservations;
    } else {
      yellow_from_authorized_green_ = false;
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

  void reset()
  {
    previous_ = Observation::Missing;
    yellow_from_authorized_green_ = false;
    green_confirmations_ = 0;
    green_authorized_ = false;
    last_source_.reset();
    last_now_.reset();
  }

private:
  Observation previous_{Observation::Missing};
  bool yellow_from_authorized_green_{false};
  std::size_t green_confirmations_{0};
  bool green_authorized_{false};
  std::optional<double> last_source_;
  std::optional<double> last_now_;
};

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
