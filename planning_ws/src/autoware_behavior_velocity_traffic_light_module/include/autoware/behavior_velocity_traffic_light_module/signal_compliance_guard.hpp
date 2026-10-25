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
