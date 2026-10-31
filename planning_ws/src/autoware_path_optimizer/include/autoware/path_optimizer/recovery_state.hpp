#pragma once

#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>

namespace autoware::path_optimizer
{
// Persistent permission to search outside the normal corridor, not the provenance
// of a single output. Solver/history resets must not reset this latch.
class RecoveryState
{
public:
  struct Parameters {
    double stopped_speed{1.0 / 3.6};
    double stopped_duration{2.0};
    double failed_duration{2.0};
    double maximum_sample_gap{0.5};
    std::size_t minimum_failed_cycles{5};
  };
  RecoveryState() = default;
  explicit RecoveryState(Parameters p) : parameters_(p) {
    if (!std::isfinite(p.stopped_speed) || p.stopped_speed < 0 ||
        !std::isfinite(p.stopped_duration) || p.stopped_duration <= 0 ||
        !std::isfinite(p.failed_duration) || p.failed_duration <= 0 ||
        !std::isfinite(p.maximum_sample_gap) || p.maximum_sample_gap <= 0 ||
        p.minimum_failed_cycles < 2) throw std::invalid_argument("invalid recovery gate");
  }
  bool active() const { return active_; }
  std::size_t failedCycles() const { return failed_cycles_; }
  void observe(double time, double speed, bool valid, bool permitted) {
    sample_valid_ = std::isfinite(time) && std::isfinite(speed) && valid;
    if (!sample_valid_ || !permitted ||
        (last_ && (time < *last_ || time - *last_ > parameters_.maximum_sample_gap))) {
      clearEvidence();
      if (std::isfinite(time)) last_ = time;
      return;  // An outage or clock reset cannot manufacture a stopped failure.
    }
    last_ = time;
    sample_valid_ = true;
    if (std::abs(speed) <= parameters_.stopped_speed) {
      if (!stopped_since_) stopped_since_ = time;
    } else stopped_since_.reset();
  }
  void normalResult(double time, bool fresh_success, bool failed_to_stop, bool permitted) {
    if (fresh_success) {
      if (sample_valid_) active_ = false;
      clearEvidence();
      return;
    }
    if (active_) return;  // No timeout, speed threshold, or retry count unlatches recovery.
    if (!sample_valid_ || !last_ || !permitted || !failed_to_stop || !std::isfinite(time) || time != *last_) {
      failed_since_.reset(); failed_cycles_ = 0;
      return;
    }
    if (!failed_since_) failed_since_ = time;
    ++failed_cycles_;
    active_ = stopped_since_ && time - *stopped_since_ >= parameters_.stopped_duration &&
      time - *failed_since_ >= parameters_.failed_duration &&
      failed_cycles_ >= parameters_.minimum_failed_cycles;
  }
  double stoppedSeconds(double time) const { return stopped_since_ ? time - *stopped_since_ : 0; }
  double failedSeconds(double time) const { return failed_since_ ? time - *failed_since_ : 0; }
  void clearEvidence() { stopped_since_.reset(); failed_since_.reset(); failed_cycles_ = 0; }
private:
  Parameters parameters_;
  bool active_{false};
  bool sample_valid_{false};
  std::optional<double> last_, stopped_since_, failed_since_;
  std::size_t failed_cycles_{0};
};
}  // namespace autoware::path_optimizer
