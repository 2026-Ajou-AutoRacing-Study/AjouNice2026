#ifndef AUTOWARE_LC_MEASURED_MOTION_POLICY_HPP
#define AUTOWARE_LC_MEASURED_MOTION_POLICY_HPP
#include <cmath>
#include <cstddef>
namespace autoware::behavior_path_planner::utils::lane_change {
// Only an additional future measured-motion hypothesis may be diagnostic.
// Invalid timestamps and the present state remain hard constraints.
inline bool measured_failure_is_diagnostic(
  const bool enabled, const size_t hypothesis, const double time)
{
  return enabled && hypothesis == 1 && std::isfinite(time) && time > 1e-6;
}
}
#endif
