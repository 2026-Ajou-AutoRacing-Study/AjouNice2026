#pragma once

#include "autoware/behavior_path_lane_change_module/structs/data.hpp"
#include <chrono>
#include <mutex>
#include <optional>
#include <stdexcept>

namespace autoware::behavior_path_planner::utils::lane_change {
// Incomplete constructor inputs, never an admitted LaneChangePath.
struct GenerationRejection : std::logic_error {
  PathWithLaneId prepare, reference;
  // Value snapshot only after the diagnostic rate claim; never a shared mutable input.
  std::optional<PathWithLaneId> upstream;
  ShiftedPath shifted;
  LaneChangeInfo info;
  double yaw, threshold;
  GenerationRejection(const std::string & reason, const PathWithLaneId & p,
    const ShiftedPath & s, const PathWithLaneId & r, const LaneChangeInfo & i,
    double y, double limit, const PathWithLaneId * input_upstream = nullptr)
  : std::logic_error(reason), prepare(p), reference(r), shifted(s), info(i),
    yaw(y), threshold(limit) {
      if (input_upstream) upstream = *input_upstream;
    }
};

class GenerationDiagnosticRate {
  std::mutex mutex_;
  double last_ = -1.0;
public:
  bool claim(bool enabled, double now) {
    if (!enabled) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    if (last_ >= 0.0 && now - last_ < 1.0) return false;
    last_ = now;
    return true;
  }
};

template<class Diagnostic> void generation_diagnostic_nonfatal(Diagnostic diagnostic) noexcept {
  try { diagnostic(); } catch (...) { /* Diagnostics cannot change rejection/continue. */ }
}

[[noreturn]] inline void reject_generation_yaw(bool capture, const std::string & reason,
  const PathWithLaneId & prepare, const ShiftedPath & shifted,
  const PathWithLaneId & reference, const LaneChangeInfo & info, double yaw, double threshold,
  const PathWithLaneId * upstream = nullptr)
{
  if (capture) {
    try { throw GenerationRejection(reason, prepare, shifted, reference, info, yaw, threshold, upstream); }
    catch (const GenerationRejection &) { throw; }
    catch (...) { /* Payload allocation failure retains the original rejection. */ }
  }
  throw std::logic_error(reason);
}
}  // namespace autoware::behavior_path_planner::utils::lane_change
