#pragma once

#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <geometry_msgs/Pose.h>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner {
struct GoalCandidateCore { size_t id{}; geometry_msgs::Pose goal_pose{}; };
class PullOverPathCore {
public:
  static std::optional<PullOverPathCore> create(
    const std::vector<autoware_internal_planning_msgs::PathWithLaneId> & partial_paths,
    const geometry_msgs::Pose & start_pose, const GoalCandidateCore & goal,
    const std::vector<std::pair<double, double>> & terminal_pairs);
  const autoware_internal_planning_msgs::PathWithLaneId & fullPath() const { return full_path_; }
  const autoware_internal_planning_msgs::PathWithLaneId & parkingPath() const { return parking_path_; }
  size_t goalId() const { return goal_.id; }
  size_t pathIndex() const { return path_index_; }
  bool incrementPathIndex();
  const autoware_internal_planning_msgs::PathWithLaneId & currentPath() const;
  std::pair<double, double> terminalPair() const;
  double fullPathMaxCurvature() const { return full_path_max_curvature_; }
  double parkingPathCurvatureTotalDerivative() const { return parking_path_curvature_total_derivative_; }
private:
  PullOverPathCore(
    std::vector<autoware_internal_planning_msgs::PathWithLaneId> partial_paths,
    autoware_internal_planning_msgs::PathWithLaneId full_path,
    autoware_internal_planning_msgs::PathWithLaneId parking_path, GoalCandidateCore goal,
    std::vector<std::pair<double, double>> terminal_pairs, double full_path_max_curvature,
    double parking_path_curvature_total_derivative);
  std::vector<autoware_internal_planning_msgs::PathWithLaneId> partial_paths_;
  autoware_internal_planning_msgs::PathWithLaneId full_path_;
  autoware_internal_planning_msgs::PathWithLaneId parking_path_;
  GoalCandidateCore goal_;
  std::vector<std::pair<double, double>> terminal_pairs_;
  double full_path_max_curvature_{};
  double parking_path_curvature_total_derivative_{};
  size_t path_index_{};
};
}
