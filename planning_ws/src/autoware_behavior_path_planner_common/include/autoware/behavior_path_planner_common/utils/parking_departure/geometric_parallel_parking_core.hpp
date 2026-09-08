#pragma once

#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <geometry_msgs/Pose.h>

#include <vector>

namespace autoware::behavior_path_planner::utils::parking_departure
{
// Source-equivalent geometric kernel used by GeometricParallelParking's two-turn paths.
autoware_internal_planning_msgs::PathPointWithLaneId generateArcPathPoint(
  const geometry_msgs::Pose & center, double radius, double yaw, bool is_left_turn,
  bool is_forward);
autoware_internal_planning_msgs::PathWithLaneId generateArcPath(
  const geometry_msgs::Pose & center, double radius, double start_yaw, double end_yaw,
  double arc_path_interval, bool is_left_turn, bool is_forward);
void setVelocityToArcPaths(
  std::vector<autoware_internal_planning_msgs::PathWithLaneId> & arc_paths,
  double velocity, bool set_stop_end);

// State portion of GeometricParallelParking; planning trials populate the two path sequences.
class GeometricParallelParkingPathState
{
public:
  void setPaths(std::vector<autoware_internal_planning_msgs::PathWithLaneId> paths);
  void setArcPaths(std::vector<autoware_internal_planning_msgs::PathWithLaneId> arc_paths);
  void clearPaths();
  void incrementPathIndex();
  bool isParking() const;
  autoware_internal_planning_msgs::PathWithLaneId getPathByIdx(size_t index) const;
  autoware_internal_planning_msgs::PathWithLaneId getCurrentPath() const;
  autoware_internal_planning_msgs::PathWithLaneId getFullPath() const;
  autoware_internal_planning_msgs::PathWithLaneId getArcPath() const;

private:
  std::vector<autoware_internal_planning_msgs::PathWithLaneId> arc_paths_;
  std::vector<autoware_internal_planning_msgs::PathWithLaneId> paths_;
  size_t current_path_idx_{0};
};
}  // namespace autoware::behavior_path_planner::utils::parking_departure
