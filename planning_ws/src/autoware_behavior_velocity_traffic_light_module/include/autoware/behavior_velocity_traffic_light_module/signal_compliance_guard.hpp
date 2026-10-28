// Copyright 2026 AjouNice2026 contributors
// Licensed under the Apache License, Version 2.0.
#ifndef AUTOWARE_BEHAVIOR_VELOCITY_TRAFFIC_LIGHT_MODULE__SIGNAL_COMPLIANCE_GUARD_HPP_
#define AUTOWARE_BEHAVIOR_VELOCITY_TRAFFIC_LIGHT_MODULE__SIGNAL_COMPLIANCE_GUARD_HPP_

#include <algorithm>
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

// Competition-only initial-observation policy: a fresh unreadable signal is
// not evidence of green, but it need not insert a stop point until the first
// readable lamp on this approach. After any readable lamp, UNKNOWN stops.
inline bool deferUnseenUntilObserved(bool enabled, bool seen_signal_on_approach,
                                     Observation observation)
{
  return enabled && !seen_signal_on_approach && observation == Observation::Unknown;
}

// A fresh empty observation is not evidence of green. It may leave the path
// open only before the first readable lamp on this approach and only while
// there is room to brake after entering the observation zone.
inline bool deferFarUnseenUnknown(
  bool enabled, bool seen_signal_on_approach, bool empty_observation,
  double distance_to_stop_point, double stopping_distance, double speed,
  double minimum_distance, double response_margin_sec)
{
  if (!enabled || seen_signal_on_approach || !empty_observation ||
      !std::isfinite(distance_to_stop_point) || !std::isfinite(stopping_distance) ||
      !std::isfinite(speed) || stopping_distance < 0.0 || speed < 0.0) {
    return false;
  }
  const double decision_distance = std::max(
    minimum_distance, stopping_distance + speed * response_margin_sec + 5.0);
  return distance_to_stop_point > decision_distance;
}

// Competition policy: after a previously observed signal's virtual Lanelet line,
// pure unreadability must not introduce another stop, even if already stopped.
// This is a geometry/history proxy, not explicit image-FOV evidence. Readable
// RED/YELLOW and known-color ambiguity still use the ordinary guarded decision.
inline bool deferTerminalUnknownAfterVirtualLine(
  bool enabled, bool previously_observed, Observation observation,
  bool has_color_evidence, std::optional<double> front_distance, bool fresh)
{
  return enabled && previously_observed && fresh && observation == Observation::Unknown &&
    !has_color_evidence && front_distance && std::isfinite(*front_distance) && *front_distance < 0.0;
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

  void setUnknownGraceSec(double seconds)
  {
    if (!std::isfinite(seconds) || seconds < 0.0 || seconds > 0.3) {
      throw std::invalid_argument("unknown grace must be finite in [0,0.3] seconds");
    }
    if (unknown_grace_sec_ != seconds) {
      reset();
      unknown_grace_sec_ = seconds;
    }
  }

  bool observe(Observation observation, double source_sec, double now_sec, double timeout_sec,
               bool has_color_evidence = true)
  {
    // Default zero preserves the original policy, including its reset behavior.
    // UNKNOWN grace must explicitly opt in AND identify genuinely unreadable
    // wire observations. A known color with unknown shape/status never qualifies.
    if (unknown_grace_sec_ > 0.0) {
      return observeWithUnknownGrace(
        observation, source_sec, now_sec, timeout_sec, has_color_evidence);
    }
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
  bool unknownGraceActive() const { return unknown_grace_active_; }
  double unknownGraceSec() const { return unknown_grace_sec_; }
  std::optional<double> lastRealGreenSourceSec() const { return last_real_green_source_; }
  std::optional<double> lastRealGreenProcessedSec() const { return last_real_green_processed_; }

  void reset()
  {
    previous_ = Observation::Missing;
    yellow_from_authorized_green_ = false;
    green_confirmations_ = 0;
    green_authorized_ = false;
    last_source_.reset();
    last_now_.reset();
    yellow_start_lower_bound_sec_.reset();
    last_real_green_source_.reset();
    last_real_green_processed_.reset();
    input_source_highwater_.reset();
    input_now_highwater_.reset();
    unknown_grace_active_ = false;
  }

private:
  // Clear authorization without forgetting an already consumed input stamp.
  // In particular, G -> UNKNOWN(newer stamp) -> G(older/duplicate stamp) must
  // never renew either the grant or the original green's processing lease.
  void clearAuthorization()
  {
    previous_ = Observation::Missing;
    yellow_from_authorized_green_ = false;
    green_confirmations_ = 0;
    green_authorized_ = false;
    last_source_.reset();
    last_now_.reset();
    yellow_start_lower_bound_sec_.reset();
    last_real_green_source_.reset();
    last_real_green_processed_.reset();
    unknown_grace_active_ = false;
  }

  bool observeWithUnknownGrace(
    Observation observation, double source_sec, double now_sec, double timeout_sec,
    bool has_color_evidence)
  {
    unknown_grace_active_ = false;
    const bool valid_now = std::isfinite(now_sec) && now_sec > 0.0;
    const bool fresh = std::isfinite(source_sec) && source_sec > 0.0 && valid_now &&
      std::isfinite(timeout_sec) && timeout_sec > 0.0 &&
      now_sec >= source_sec && now_sec - source_sec <= timeout_sec;
    if (valid_now && input_now_highwater_ && now_sec < *input_now_highwater_) {
      // A new clock epoch must first STOP, even with confirmation count one.
      reset();
      input_now_highwater_ = now_sec;
      if (fresh) input_source_highwater_ = source_sec;
      return true;
    }
    if (valid_now) input_now_highwater_ = now_sec;
    if (!fresh || observation == Observation::Missing || observation == Observation::Stale) {
      clearAuthorization();
      return true;
    }
    if (input_source_highwater_ && source_sec < *input_source_highwater_) {
      clearAuthorization();
      return true;
    }
    const bool distinct = !input_source_highwater_ || source_sec > *input_source_highwater_;
    input_source_highwater_ = source_sec;
    if (observation == Observation::Unknown) {
      const bool in_original_lease = last_real_green_source_ && last_real_green_processed_ &&
        now_sec >= *last_real_green_source_ && now_sec >= *last_real_green_processed_ &&
        now_sec - *last_real_green_source_ <= unknown_grace_sec_ &&
        now_sec - *last_real_green_processed_ <= unknown_grace_sec_;
      if (!has_color_evidence && previous_ == Observation::AuthorizedGreen &&
          green_authorized_ && in_original_lease) {
        // Retain only the previous grant. Do not turn UNKNOWN into a new green,
        // increment confirmations, refresh its source/lease, or move Y's lower
        // onset bound to this UNKNOWN image's newer timestamp.
        unknown_grace_active_ = true;
        return false;
      }
      clearAuthorization();
      return true;
    }
    if (last_source_ && now_sec - *last_source_ > timeout_sec) clearAuthorization();
    if (observation == Observation::Yellow) {
      if (previous_ != Observation::Yellow) {
        yellow_from_authorized_green_ = previous_ == Observation::AuthorizedGreen &&
          green_authorized_ && last_real_green_source_ && source_sec > *last_real_green_source_ &&
          now_sec - *last_real_green_source_ <= timeout_sec;
        yellow_start_lower_bound_sec_ = yellow_from_authorized_green_ ? last_real_green_source_ : std::nullopt;
      }
      green_confirmations_ = 0;
      green_authorized_ = false;
      last_real_green_source_.reset();
      last_real_green_processed_.reset();
    } else if (observation == Observation::AuthorizedGreen) {
      yellow_from_authorized_green_ = false;
      yellow_start_lower_bound_sec_.reset();
      if (previous_ != Observation::AuthorizedGreen) {
        green_confirmations_ = 0;
        green_authorized_ = false;
      }
      if (distinct && green_confirmations_ < required_green_observations_) ++green_confirmations_;
      green_authorized_ = green_confirmations_ >= required_green_observations_;
      if (distinct && green_authorized_) {
        last_real_green_source_ = source_sec;
        last_real_green_processed_ = now_sec;
      }
    } else {
      clearAuthorization();
    }
    previous_ = observation;
    last_source_ = source_sec;
    last_now_ = now_sec;
    return observation != Observation::AuthorizedGreen || !green_authorized_;
  }

  Observation previous_{Observation::Missing};
  bool yellow_from_authorized_green_{false};
  std::size_t green_confirmations_{0};
  std::size_t required_green_observations_{requiredGreenObservations};
  bool green_authorized_{false};
  std::optional<double> last_source_;
  std::optional<double> last_now_;
  std::optional<double> yellow_start_lower_bound_sec_;
  double unknown_grace_sec_{0.0};
  bool unknown_grace_active_{false};
  std::optional<double> last_real_green_source_;
  std::optional<double> last_real_green_processed_;
  std::optional<double> input_source_highwater_;
  std::optional<double> input_now_highwater_;
};

// Candidate decision-line distance. The explicit model starts at a0=0, coasts
// for delay_sec, ramps acceleration with -jerk_mps3 until either stopping or
// reaching -deceleration_mps2, then holds that deceleration until zero speed.
// This is not measured braking capability, a symmetric comfort S-curve, or a
// substitute for camera/processing/actuator latency.
inline std::optional<double> jerkStoppingDistance(
  double velocity_mps, double deceleration_mps2, double jerk_mps3, double delay_sec)
{
  // Signed odometry can jitter slightly below zero while parked. Treat the
  // inclusive [-0.1,0) m/s standstill band as zero, never as reverse braking.
  // A true reverse speed below this band remains unsupported/invalid.
  if (!std::isfinite(velocity_mps) || velocity_mps < -0.1 ||
      !std::isfinite(deceleration_mps2) || deceleration_mps2 <= 0.0 ||
      !std::isfinite(jerk_mps3) || jerk_mps3 <= 0.0 ||
      !std::isfinite(delay_sec) || delay_sec < 0.0) {
    return std::nullopt;
  }
  const long double v = std::max(0.0, velocity_mps);
  const long double b = deceleration_mps2;
  const long double j = jerk_mps3;
  const long double t = std::min(b / j, std::sqrt(2.0L * v / j));
  const long double residual_v = std::max(0.0L, v - j * t * t / 2.0L);
  const long double distance = v * delay_sec + v * t - j * t * t * t / 6.0L +
    residual_v * residual_v / (2.0L * b);
  if (!std::isfinite(distance) || distance < 0.0L ||
      distance > std::numeric_limits<double>::max()) {
    return std::nullopt;
  }
  return static_cast<double>(distance);
}

enum class DecisionLineState { Approach, Stop, Go };

inline const char * decisionLineStateName(DecisionLineState state)
{
  switch (state) {
    case DecisionLineState::Approach: return "approach";
    case DecisionLineState::Stop: return "stop";
    case DecisionLineState::Go: return "committed_go";
  }
  return "invalid";
}

struct DecisionLineDecision
{
  bool require_stop{true};
  DecisionLineState state{DecisionLineState::Approach};
  const char * reason{"reset"};
  double stopping_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double stopping_target_distance_m{std::numeric_limits<double>::quiet_NaN()};
  bool committed_go{false};
};

// Opt-in competition policy. Observation::AuthorizedGreen is already specific
// to the route direction (including D's R+left-arrow), not the raw green lamp.
// A fresh moving grant at/inside the dynamic decision boundary commits GO;
// later R/Y do not reverse that decision until an actual stop/reset. This
// deliberate one-shot behavior does NOT guarantee an original-MGeo-line legal
// crossing if red starts before the vehicle reaches that separate grading line.
// The owning scene must reset on mission/route reset and retire the completed
// mission at the original line. A crossed virtual camera-friendly stop target
// alone must never erase a pending STOP.
class DecisionLineLatch
{
public:
  DecisionLineDecision update(
    Observation observation, bool fresh, bool guard_requires_stop,
    double velocity_mps, bool vehicle_stopped, double target_distance_m,
    double stopping_distance_m, double source_sec, double now_sec,
    bool defer_initial_unseen_unknown = false)
  {
    const auto finish = [&](bool require_stop, const char * reason) {
      last_decision_ = {require_stop, state_, reason, stopping_distance_m,
                        target_distance_m, state_ == DecisionLineState::Go};
      return last_decision_;
    };
    if (!std::isfinite(now_sec) || now_sec <= 0.0) {
      reset();
      state_ = DecisionLineState::Stop;
      return finish(true, "invalid_clock");
    }
    const bool valid_source = std::isfinite(source_sec) && source_sec > 0.0;
    const bool rewound = (last_now_sec_ && now_sec < *last_now_sec_) ||
      (fresh && valid_source && last_source_sec_ && source_sec < *last_source_sec_);
    if (rewound) {
      reset();
      state_ = DecisionLineState::Stop;
      last_now_sec_ = now_sec;
      if (fresh && valid_source && source_sec <= now_sec) last_source_sec_ = source_sec;
      return finish(true, "clock_rewind");
    }
    last_now_sec_ = now_sec;
    const bool usable_source = fresh && valid_source && source_sec <= now_sec;
    if (usable_source) last_source_sec_ = source_sec;
    if (!std::isfinite(velocity_mps) || velocity_mps < -moving_threshold_mps) {
      // The caller may explicitly retain the competition's existing pre-first-
      // lamp UNKNOWN policy. A finite negative signed-speed sample is not a
      // red/yellow observation and must not create permanent color-STOP history
      // for an unseen future signal. Do not fabricate a braking distance or a
      // directional grant, broaden the standstill clamp, release an existing
      // STOP, or preserve an already committed GO through this exception.
      if (std::isfinite(velocity_mps) && defer_initial_unseen_unknown && usable_source &&
          observation == Observation::Unknown && !guard_requires_stop &&
          state_ == DecisionLineState::Approach) {
        return finish(false, "defer_unseen_negative_velocity");
      }
      state_ = DecisionLineState::Stop;
      return finish(true, "invalid_velocity");
    }
    // Match jerkStoppingDistance's inclusive parked-odometry noise band. This
    // must not create a sticky color STOP before any signal has been observed.
    velocity_mps = std::max(0.0, velocity_mps);
    const bool moving = !vehicle_stopped && velocity_mps > moving_threshold_mps;
    if (state_ == DecisionLineState::Go && moving) {
      // Missing geometry/UNKNOWN do not fabricate a new grant, but neither do
      // they revoke an already committed one-shot decision.
      return finish(false, "retain_committed_go");
    }
    const bool stopped_after_commit = state_ == DecisionLineState::Go;
    if (stopped_after_commit) state_ = DecisionLineState::Approach;
    const bool authorized = usable_source &&
      observation == Observation::AuthorizedGreen && !guard_requires_stop;
    if (state_ == DecisionLineState::Stop && !authorized) {
      return finish(true, "retain_stop_until_fresh_authorization");
    }
    if (state_ == DecisionLineState::Stop && authorized) {
      state_ = DecisionLineState::Approach;
    }
    if (guard_requires_stop || (stopped_after_commit && !authorized)) {
      state_ = DecisionLineState::Stop;
      return finish(true, "stop_selected");
    }
    const bool valid_geometry = std::isfinite(target_distance_m) &&
      std::isfinite(stopping_distance_m) && stopping_distance_m >= 0.0;
    if (authorized && moving && valid_geometry && target_distance_m <= stopping_distance_m) {
      state_ = DecisionLineState::Go;
      return finish(false, "decision_boundary_authorized_go");
    }
    // Before commitment, keep the existing guard's decision, including its
    // bounded UNKNOWN grace/initial-unseen policy. These are not new grants.
    return finish(false, authorized ? "authorized_before_boundary_or_stationary" :
                                    "approach_without_new_authorization");
  }

  void reset()
  {
    state_ = DecisionLineState::Approach;
    last_now_sec_.reset();
    last_source_sec_.reset();
    last_decision_ = {};
  }

  DecisionLineState state() const { return state_; }
  bool pendingStop() const { return state_ == DecisionLineState::Stop; }
  const DecisionLineDecision & lastDecision() const { return last_decision_; }

private:
  static constexpr double moving_threshold_mps = 0.1;
  DecisionLineState state_{DecisionLineState::Approach};
  std::optional<double> last_now_sec_;
  std::optional<double> last_source_sec_;
  DecisionLineDecision last_decision_;
};

struct LaneletYellowDecision
{
  bool allow{false};
  const char * reason{"disabled"};
  double stopping_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double stopping_target_distance_m{std::numeric_limits<double>::quiet_NaN()};
};

// Competition yellow policy: retain a previously granted, moving approach only
// when the camera-friendly Lanelet stopping target cannot be reached before
// stopping under this explicitly simplified model. This is not a guarantee of
// measured braking capability, lamp duration, or a legal original-line crossing.
// The target already includes the front-bumper offset and stop margin. Do not
// subtract the wheelbase/front-axle offset from it a second time.
inline LaneletYellowDecision laneletYellowDecision(
  bool enabled, bool transition_eligible, bool previous_stop, double velocity_mps,
  double slow_velocity_mps, double response_delay_sec, double deceleration_mps2,
  double signed_stopping_target_distance_m)
{
  LaneletYellowDecision result;
  result.stopping_target_distance_m = signed_stopping_target_distance_m;
  if (!enabled) return result;
  result.reason = "invalid_input";
  if (!std::isfinite(velocity_mps) || velocity_mps < 0.0 ||
      !std::isfinite(slow_velocity_mps) || slow_velocity_mps <= 0.0 ||
      !std::isfinite(response_delay_sec) || response_delay_sec < 0.0 ||
      !std::isfinite(deceleration_mps2) || deceleration_mps2 <= 0.0 ||
      !std::isfinite(signed_stopping_target_distance_m)) {
    return result;
  }
  result.stopping_distance_m = velocity_mps * response_delay_sec +
    velocity_mps * velocity_mps / (2.0 * deceleration_mps2);
  if (!std::isfinite(result.stopping_distance_m)) return result;
  result.reason = "no_observed_authorized_transition";
  if (!transition_eligible) return result;
  result.reason = "already_stopped_or_slow";
  if (previous_stop || velocity_mps < slow_velocity_mps) return result;
  // Equality means the model can stop at the target, so retain STOP on the tie.
  result.allow = result.stopping_distance_m > signed_stopping_target_distance_m;
  result.reason = result.allow ? "cannot_stop_before_lanelet_target" : "lanelet_target_stoppable";
  return result;
}

// Historical pure helper, retained only for old experiment reproduction/tests.
// Production scenes no longer call this duration/ETA policy.
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
