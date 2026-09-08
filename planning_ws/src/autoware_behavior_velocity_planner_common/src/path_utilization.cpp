#include <autoware/behavior_velocity_planner_common/utilization/path_utilization.hpp>
#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <ros/console.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace autoware::behavior_velocity_planner
{
// Original path_utilization.cpp:26-38; preserve all resampling options.
bool splineInterpolate(
  const autoware_internal_planning_msgs::PathWithLaneId & input, const double interval,
  autoware_internal_planning_msgs::PathWithLaneId & output,
  [[maybe_unused]] const std::string & logger)
{
  if (input.points.size() < 2) {
    ROS_DEBUG("Do not interpolate because path size is 1.");
    return false;
  }
  output = autoware::motion_utils::resamplePath(input, interval, false, true, true, false);
  return true;
}

autoware_planning_msgs::Path interpolatePath(
  const autoware_planning_msgs::Path & path, const double length, const double interval)
{
  if (path.points.size() < 2U) return path;

  const double path_length = std::min(length, autoware::motion_utils::calcArcLength(path.points));
  std::vector<double> input_arc_lengths;
  input_arc_lengths.reserve(path.points.size());
  input_arc_lengths.push_back(0.0);
  for (size_t i = 1U; i < path.points.size(); ++i) {
    const auto & previous = path.points.at(i - 1U).pose.position;
    const auto & current = path.points.at(i).pose.position;
    const double dx = current.x - previous.x;
    const double dy = current.y - previous.y;
    const double arc_length = input_arc_lengths.back() + std::hypot(dx, dy);
    if (arc_length > path_length) break;
    input_arc_lengths.push_back(arc_length);
  }

  const double resample_length = std::min(path_length, input_arc_lengths.back());
  if (std::fabs(input_arc_lengths.back() - resample_length) < 0.01) {
    input_arc_lengths.back() = resample_length;
  } else {
    input_arc_lengths.push_back(resample_length);
  }

  std::vector<double> output_arc_lengths = input_arc_lengths;
  for (double s = 0.0; s < resample_length; s += interval) {
    if (std::none_of(output_arc_lengths.begin(), output_arc_lengths.end(), [s](const double value) {
          return std::fabs(value - s) < 0.01;
        })) {
      output_arc_lengths.push_back(s);
    }
  }
  std::sort(output_arc_lengths.begin(), output_arc_lengths.end());
  return autoware::motion_utils::resamplePath(path, output_arc_lengths);
}

autoware_planning_msgs::Path filterLitterPathPoint(const autoware_planning_msgs::Path & path)
{
  autoware_planning_msgs::Path filtered_path;
  constexpr double epsilon = 0.01;
  size_t latest_id = 0;
  for (size_t i = 0; i < path.points.size(); ++i) {
    double dist = 0.0;
    if (i != 0) {
      const double x = path.points.at(i).pose.position.x - path.points.at(latest_id).pose.position.x;
      const double y = path.points.at(i).pose.position.y - path.points.at(latest_id).pose.position.y;
      dist = std::sqrt(x * x + y * y);
    }
    if (i == 0 || epsilon < dist) {
      latest_id = i;
      filtered_path.points.push_back(path.points.at(latest_id));
    } else {
      filtered_path.points.back().longitudinal_velocity_mps = std::min(
        filtered_path.points.back().longitudinal_velocity_mps,
        path.points.at(i).longitudinal_velocity_mps);
    }
  }
  return filtered_path;
}

autoware_planning_msgs::Path filterStopPathPoint(const autoware_planning_msgs::Path & path)
{
  auto filtered_path = path;
  bool found_stop = false;
  for (auto & point : filtered_path.points) {
    if (std::fabs(point.longitudinal_velocity_mps) < 0.01) found_stop = true;
    if (found_stop) point.longitudinal_velocity_mps = 0.0;
  }
  return filtered_path;
}
}  // namespace autoware::behavior_velocity_planner
