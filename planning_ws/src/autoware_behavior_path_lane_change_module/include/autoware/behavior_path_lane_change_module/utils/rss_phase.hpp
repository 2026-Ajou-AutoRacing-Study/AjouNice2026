#ifndef AUTOWARE_BEHAVIOR_PATH_LANE_CHANGE_MODULE_UTILS_RSS_PHASE_HPP_
#define AUTOWARE_BEHAVIOR_PATH_LANE_CHANGE_MODULE_UTILS_RSS_PHASE_HPP_

#include <autoware/behavior_path_planner_common/utils/path_safety_checker/parameters_core.hpp>
#include <cmath>
#include <string>

namespace autoware::behavior_path_planner::utils::lane_change
{
// Restore upstream scene.cpp is_colliding's per-object-sample selector:
// time < prepare_duration => prepare RSS, equality => lane-changing RSS.
// Do not resample, rebase time, truncate ego predictions, or drop object samples.
// The core callback receives one original predicted path's contiguous phase;
// preserve its original full path in diagnostics if that phase is unsafe.
template<class Object, class EgoPath, class Check>
bool check_rss_phases(
  const Object & object, const EgoPath & ego_path, const double prepare_duration,
  Check check, std::string & reason)
{
  if (!std::isfinite(prepare_duration) || prepare_duration < 0.0) {
    reason = "rss_phase_invalid_prepare_duration";
    return false;
  }
  const auto valid_times = [](const auto & path) {
    if (path.empty()) return false;
    for (size_t i = 0; i < path.size(); ++i) {
      if (!std::isfinite(path[i].time) || path[i].time < 0.0 ||
          (i && path[i].time <= path[i - 1].time)) return false;
    }
    return true;
  };
  if (!valid_times(ego_path)) {
    reason = "rss_phase_invalid_ego_times";
    return false;
  }
  if (object.predicted_paths.empty()) {
    reason = "rss_phase_missing_object_prediction";
    return false;
  }
  // Validate every hypothesis before a partial safe result can be emitted.
  for (const auto & prediction : object.predicted_paths) {
    if (!valid_times(prediction.path)) {
      reason = "rss_phase_invalid_object_times";
      return false;
    }
  }
  Object phase_object = object;
  phase_object.predicted_paths.clear();
  for (const auto & prediction : object.predicted_paths) {
    size_t split = 0;
    while (split < prediction.path.size() &&
           prediction.path[split].time < prepare_duration) ++split;
    for (const bool prepare : {true, false}) {
      const size_t begin = prepare ? 0 : split;
      const size_t end = prepare ? split : prediction.path.size();
      if (begin == end) continue;
      auto phase = prediction;
      phase.path.assign(prediction.path.begin() + begin, prediction.path.begin() + end);
      phase_object.predicted_paths = {std::move(phase)};
      if (!check(phase_object, prepare, prediction)) {
        reason = prepare ? "rss_prepare_unsafe" : "rss_lane_changing_unsafe";
        return false;  // Never overwrite this unsafe diagnostic with a later safe phase.
      }
    }
  }
  reason = "safe";
  return true;
}
}  // namespace autoware::behavior_path_planner::utils::lane_change
#endif
