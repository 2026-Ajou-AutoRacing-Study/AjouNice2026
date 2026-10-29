// Copyright 2026 AjouNice2026
// SPDX-License-Identifier: Apache-2.0

#ifndef GO_AFTER_STOP_LATCH_HPP_
#define GO_AFTER_STOP_LATCH_HPP_

#include <cmath>
#include <optional>

namespace autoware::behavior_velocity_planner
{

// Experimental policy, scoped to one RoundaboutModule instance. This deliberately
// retains GO even if a later collision check requests STOP. It does not change
// collision detection, other modules' trajectories, or controller commands.
class GoAfterStopLatch
{
public:
  bool update(
    const bool enabled, const double measured_velocity_mps, const bool safe_go,
    const double x, const double y, const double now_sec)
  {
    if (
      !enabled || !std::isfinite(measured_velocity_mps) || !std::isfinite(x) ||
      !std::isfinite(y) || !std::isfinite(now_sec)) {
      reset();
      return false;
    }

    // A simulator teleport or clock rewind starts a new encounter even when the
    // scene manager has retained this module for the same entry lanelet.
    if (
      previous_sample_ &&
      (now_sec < previous_sample_->time ||
       std::hypot(x - previous_sample_->x, y - previous_sample_->y) > 7.0)) {
      reset();
    }
    previous_sample_ = Sample{x, y, now_sec};

    // Observe a physical stop while roundabout STOP is active. Merely starting
    // the scenario stationary with an already-safe path must not commit entry.
    // A zero target alone must not arm while the vehicle is still moving.
    if (!safe_go && std::abs(measured_velocity_mps) <= 0.01) {
      stopped_observed_ = true;
    }
    if (stopped_observed_ && safe_go) {
      latched_ = true;
    }
    return latched_;
  }

  bool stoppedObserved() const { return stopped_observed_; }
  bool isLatched() const { return latched_; }

  void reset()
  {
    stopped_observed_ = false;
    latched_ = false;
    previous_sample_.reset();
  }

private:
  struct Sample
  {
    double x;
    double y;
    double time;
  };
  bool stopped_observed_{false};
  bool latched_{false};
  std::optional<Sample> previous_sample_;
};

}  // namespace autoware::behavior_velocity_planner

#endif  // GO_AFTER_STOP_LATCH_HPP_
