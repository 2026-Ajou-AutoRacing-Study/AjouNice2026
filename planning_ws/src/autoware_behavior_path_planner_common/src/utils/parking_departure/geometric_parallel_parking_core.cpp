#include <autoware/behavior_path_planner_common/utils/parking_departure/geometric_parallel_parking_core.hpp>

#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
#include <stdexcept>

namespace autoware::behavior_path_planner::utils::parking_departure
{
namespace
{
double normalizeRadian(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

geometry_msgs::Pose transformPose(const geometry_msgs::Pose & local, const geometry_msgs::Pose & parent)
{
  tf2::Transform parent_tf;
  tf2::fromMsg(parent, parent_tf);
  tf2::Transform local_tf;
  tf2::fromMsg(local, local_tf);
  const geometry_msgs::Transform transform = tf2::toMsg(parent_tf * local_tf);
  geometry_msgs::Pose pose;
  pose.position.x = transform.translation.x;
  pose.position.y = transform.translation.y;
  pose.position.z = transform.translation.z;
  pose.orientation = transform.rotation;
  return pose;
}

double distance2d(
  const autoware_internal_planning_msgs::PathPointWithLaneId & first,
  const autoware_internal_planning_msgs::PathPointWithLaneId & second)
{
  return std::hypot(
    first.point.pose.position.x - second.point.pose.position.x,
    first.point.pose.position.y - second.point.pose.position.y);
}
}  // namespace

autoware_internal_planning_msgs::PathPointWithLaneId generateArcPathPoint(
  const geometry_msgs::Pose & center, const double radius, const double yaw,
  const bool is_left_turn, const bool is_forward)
{
  geometry_msgs::Pose pose_center_coords;
  pose_center_coords.position.x = radius * std::cos(yaw);
  pose_center_coords.position.y = radius * std::sin(yaw);
  tf2::Quaternion quaternion;
  if ((is_left_turn && !is_forward) || (!is_left_turn && is_forward)) {
    quaternion.setRPY(0.0, 0.0, normalizeRadian(yaw - M_PI_2));
  } else {
    quaternion.setRPY(0.0, 0.0, normalizeRadian(yaw + M_PI_2));
  }
  pose_center_coords.orientation = tf2::toMsg(quaternion);
  autoware_internal_planning_msgs::PathPointWithLaneId point;
  point.point.pose = transformPose(pose_center_coords, center);
  return point;
}

autoware_internal_planning_msgs::PathWithLaneId generateArcPath(
  const geometry_msgs::Pose & center, const double radius, const double start_yaw,
  double end_yaw, const double arc_path_interval, const bool is_left_turn, const bool is_forward)
{
  if (radius <= 0.0 || arc_path_interval <= 0.0) {
    throw std::invalid_argument("arc radius and interval must be positive");
  }
  autoware_internal_planning_msgs::PathWithLaneId path;
  const double yaw_interval = arc_path_interval / radius;
  double yaw = start_yaw;
  if (is_left_turn) {
    if (end_yaw < start_yaw) end_yaw += M_PI_2;
    while (yaw < end_yaw) {
      path.points.push_back(generateArcPathPoint(center, radius, yaw, is_left_turn, is_forward));
      yaw += yaw_interval;
    }
  } else {
    if (end_yaw > start_yaw) end_yaw -= M_PI_2;
    while (yaw > end_yaw) {
      path.points.push_back(generateArcPathPoint(center, radius, yaw, is_left_turn, is_forward));
      yaw -= yaw_interval;
    }
  }
  const auto endpoint = generateArcPathPoint(center, radius, end_yaw, is_left_turn, is_forward);
  constexpr double min_distance = 0.01;
  if (path.points.empty() || distance2d(path.points.back(), endpoint) > min_distance) {
    path.points.push_back(endpoint);
  }
  return path;
}

void setVelocityToArcPaths(
  std::vector<autoware_internal_planning_msgs::PathWithLaneId> & arc_paths,
  const double velocity, const bool set_stop_end)
{
  for (auto & path : arc_paths) {
    for (size_t index = 0; index < path.points.size(); ++index) {
      path.points.at(index).point.longitudinal_velocity_mps =
        (index + 1 == path.points.size() && set_stop_end) ? 0.0F : static_cast<float>(velocity);
    }
  }
}

void GeometricParallelParkingPathState::setPaths(
  std::vector<autoware_internal_planning_msgs::PathWithLaneId> paths)
{
  paths_ = std::move(paths);
  current_path_idx_ = 0;
}

void GeometricParallelParkingPathState::setArcPaths(
  std::vector<autoware_internal_planning_msgs::PathWithLaneId> arc_paths)
{
  arc_paths_ = std::move(arc_paths);
}

void GeometricParallelParkingPathState::clearPaths()
{
  current_path_idx_ = 0;
  arc_paths_.clear();
  paths_.clear();
}

void GeometricParallelParkingPathState::incrementPathIndex()
{
  current_path_idx_ = std::min(current_path_idx_ + 1, paths_.size() - 1);
}

bool GeometricParallelParkingPathState::isParking() const
{
  return current_path_idx_ > 0;
}

autoware_internal_planning_msgs::PathWithLaneId GeometricParallelParkingPathState::getPathByIdx(
  const size_t index) const
{
  if (paths_.empty() || paths_.size() <= index) return {};
  return paths_.at(index);
}

autoware_internal_planning_msgs::PathWithLaneId GeometricParallelParkingPathState::getCurrentPath() const
{
  return paths_.at(current_path_idx_);
}

autoware_internal_planning_msgs::PathWithLaneId GeometricParallelParkingPathState::getFullPath() const
{
  autoware_internal_planning_msgs::PathWithLaneId path;
  for (const auto & partial_path : paths_) {
    path.points.insert(path.points.end(), partial_path.points.begin(), partial_path.points.end());
  }
  path.points = autoware::motion_utils::removeOverlapPoints(path.points);
  return path;
}

autoware_internal_planning_msgs::PathWithLaneId GeometricParallelParkingPathState::getArcPath() const
{
  autoware_internal_planning_msgs::PathWithLaneId path;
  for (const auto & arc_path : arc_paths_) {
    path.points.insert(path.points.end(), arc_path.points.begin(), arc_path.points.end());
  }
  return path;
}
}  // namespace autoware::behavior_path_planner::utils::parking_departure
