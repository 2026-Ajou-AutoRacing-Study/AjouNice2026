#ifndef ROUNDABOUT_PREDICTION_TIME_HPP_
#define ROUNDABOUT_PREDICTION_TIME_HPP_

#include <autoware/object_recognition_utils/predicted_path_utils.hpp>
#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

namespace autoware::behavior_velocity_planner
{
struct TimedRoundaboutPrediction
{
  autoware_perception_msgs::PredictedPath path;
  std::vector<double> times;
};

// Times are relative to the ego ETA epoch, not the object's message header.
// Never request a pose beyond the last real prediction sample. Preserve that
// endpoint even when its time is not a multiple of the resampling interval.
inline std::optional<TimedRoundaboutPrediction> alignRoundaboutPrediction(
  const autoware_perception_msgs::PredictedPath & input, const double age,
  const double duration)
{
  const double dt = input.time_step.sec + input.time_step.nanosec * 1e-9;
  if (input.path.size() < 2 || !std::isfinite(dt) || dt <= 0.0 ||
      !std::isfinite(age) || age < 0.0 || !std::isfinite(duration) || duration <= 0.0) {
    return std::nullopt;
  }
  for (const auto & p : input.path) {
    const auto & q = p.orientation;
    const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (!std::isfinite(p.position.x) || !std::isfinite(p.position.y) ||
        !std::isfinite(p.position.z) || !std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3) {
      return std::nullopt;
    }
  }
  const double last = dt * (input.path.size() - 1);
  if (!std::isfinite(last)) return std::nullopt;
  const double begin = std::max(0.0, age);
  const double end = std::min(last, age + duration);
  if (begin >= end) return std::nullopt;
  std::vector<double> samples{begin};
  // Retain original knots as well as the native 0.1 s checking resolution.
  for (size_t i = 1; i < input.path.size(); ++i) {
    const double t = dt * i;
    if (t > begin && t < end) samples.push_back(t);
  }
  for (double t = begin + 0.1; t < end; t += 0.1) samples.push_back(t);
  samples.push_back(end);
  std::sort(samples.begin(), samples.end());
  samples.erase(std::unique(samples.begin(), samples.end(),
    [](double a, double b) { return std::abs(a - b) < 1e-9; }), samples.end());
  TimedRoundaboutPrediction result;
  result.path = input;
  result.path.path.clear();
  for (const double t : samples) {
    const auto pose = autoware::object_recognition_utils::calcInterpolatedPose(input, t);
    if (!pose) return std::nullopt;
    result.path.path.push_back(*pose);
    result.times.push_back(t - age);
  }
  return result;
}

// A finite path still inside the conflict corridor has not demonstrated exit.
// The only independent clearance proof retained here is ego clearing before
// object entry (including the unchanged native margin).
inline bool unresolvedRoundaboutExit(
  const bool endpoint_intersects, const double ego_clear_time,
  const double object_enter_time, const double start_margin)
{
  return endpoint_intersects && !(ego_clear_time < object_enter_time - start_margin);
}
}  // namespace autoware::behavior_velocity_planner
#endif
