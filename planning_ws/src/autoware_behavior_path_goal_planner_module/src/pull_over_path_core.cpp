#include <autoware/behavior_path_goal_planner_module/pull_over_path_core.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <algorithm>
#include <cmath>

namespace autoware::behavior_path_planner {
PullOverPathCore::PullOverPathCore(
  std::vector<autoware_internal_planning_msgs::PathWithLaneId> partial_paths,
  autoware_internal_planning_msgs::PathWithLaneId full_path,
  autoware_internal_planning_msgs::PathWithLaneId parking_path, GoalCandidateCore goal,
  std::vector<std::pair<double, double>> terminal_pairs, const double full_path_max_curvature,
  const double parking_path_curvature_total_derivative)
: partial_paths_(std::move(partial_paths)), full_path_(std::move(full_path)),
  parking_path_(std::move(parking_path)), goal_(std::move(goal)),
  terminal_pairs_(std::move(terminal_pairs)), full_path_max_curvature_(full_path_max_curvature),
  parking_path_curvature_total_derivative_(parking_path_curvature_total_derivative) {}

std::optional<PullOverPathCore> PullOverPathCore::create(
  const std::vector<autoware_internal_planning_msgs::PathWithLaneId> & partial_paths,
  const geometry_msgs::Pose & start_pose, const GoalCandidateCore & goal,
  const std::vector<std::pair<double, double>> & terminal_pairs)
{
  if (partial_paths.empty()) return std::nullopt;
  autoware_internal_planning_msgs::PathWithLaneId concatenated;
  for (size_t i = 0; i < partial_paths.size(); ++i) {
    const auto & points = partial_paths.at(i).points;
    if (points.empty()) continue;
    concatenated.points.insert(concatenated.points.end(), points.begin() + (i == 0 ? 0 : 1), points.end());
  }
  autoware_internal_planning_msgs::PathWithLaneId full_path;
  full_path.points = autoware::motion_utils::removeOverlapPoints(concatenated.points);
  if (full_path.points.size() < 3) return std::nullopt;
  const auto start_index = autoware::motion_utils::findNearestIndex(full_path.points, start_pose.position);
  autoware_internal_planning_msgs::PathWithLaneId parking_path;
  parking_path.points.assign(full_path.points.begin() + start_index, full_path.points.end());
  if (parking_path.points.size() < 3) return std::nullopt;
  const auto curvatures = autoware::motion_utils::calcCurvature(full_path.points);
  double max_curvature = 0.0;
  for (const auto curvature : curvatures) max_curvature = std::max(max_curvature, std::abs(curvature));
  const auto parking_curvatures = autoware::motion_utils::calcCurvature(parking_path.points);
  const auto arcs = autoware::motion_utils::calcSignedArcLengthPartialSum(
    parking_path.points, 1, parking_path.points.size() - 1);
  const auto size = std::min(parking_curvatures.size(), arcs.size());
  double total_derivative = 0.0;
  for (size_t i = 0; i + 1 < size; ++i) {
    const auto distance = arcs.at(i + 1) - arcs.at(i);
    if (std::abs(distance) > 1e-6) total_derivative += std::abs((parking_curvatures.at(i + 1) - parking_curvatures.at(i)) / distance);
  }
  return PullOverPathCore(partial_paths, full_path, parking_path, goal, terminal_pairs, max_curvature, total_derivative);
}
bool PullOverPathCore::incrementPathIndex() {
  if (partial_paths_.empty() || path_index_ + 1 >= partial_paths_.size()) return false;
  ++path_index_; return true;
}
const autoware_internal_planning_msgs::PathWithLaneId & PullOverPathCore::currentPath() const {
  return partial_paths_.at(std::min(path_index_, partial_paths_.size() - 1));
}
std::pair<double, double> PullOverPathCore::terminalPair() const {
  return path_index_ < terminal_pairs_.size() ? terminal_pairs_.at(path_index_) : std::make_pair(0.0, 0.0);
}
}
