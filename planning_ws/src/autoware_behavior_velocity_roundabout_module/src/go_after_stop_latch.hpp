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
    const double x, const double y, const double now_sec,
    const double min_stop_duration_sec = 1.0)
  {
    if (
      !enabled || !std::isfinite(measured_velocity_mps) || !std::isfinite(x) ||
      !std::isfinite(y) || !std::isfinite(now_sec) ||
      !std::isfinite(min_stop_duration_sec) || min_stop_duration_sec < 0.0) {
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
    if (!stopped_observed_) {
      // Missing observations do not prove a continuous stop. The current BVP
      // runs around 10 Hz; restart an unfinished hold after a gap over 0.5 s.
      if (previous_sample_ && now_sec - previous_sample_->time > 0.5) {
        stop_start_time_sec_.reset();
        stop_duration_sec_ = 0.0;
      }

      if (std::abs(measured_velocity_mps) > 0.01) {
        stop_start_time_sec_.reset();
        stop_duration_sec_ = 0.0;
      } else {
        // Start only during roundabout STOP. If GO arrives before the hold is
        // complete, stationary observations may finish it; normal GO itself
        // is never delayed by this experimental latch.
        if (!stop_start_time_sec_ && !safe_go) {
          stop_start_time_sec_ = now_sec;
        }
        if (stop_start_time_sec_) {
          stop_duration_sec_ = now_sec - *stop_start_time_sec_;
          stopped_observed_ = stop_duration_sec_ >= min_stop_duration_sec;
        }
      }
    }
    previous_sample_ = Sample{x, y, now_sec};
    if (stopped_observed_ && safe_go) {
      latched_ = true;
    }
    return latched_;
  }

  bool stoppedObserved() const { return stopped_observed_; }
  bool isLatched() const { return latched_; }
  double stopDurationSec() const { return stop_duration_sec_; }

  void reset()
  {
    stopped_observed_ = false;
    latched_ = false;
    previous_sample_.reset();
    stop_start_time_sec_.reset();
    stop_duration_sec_ = 0.0;
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
  std::optional<double> stop_start_time_sec_;
  double stop_duration_sec_{0.0};
};

}  // namespace autoware::behavior_velocity_planner

#endif  // GO_AFTER_STOP_LATCH_HPP_
