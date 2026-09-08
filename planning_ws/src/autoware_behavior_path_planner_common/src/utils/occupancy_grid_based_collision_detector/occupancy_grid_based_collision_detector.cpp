#include <autoware/behavior_path_planner_common/utils/occupancy_grid_based_collision_detector/occupancy_grid_based_collision_detector.hpp>

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_math/normalization.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
#include <iostream>

namespace autoware::behavior_path_planner
{
int discretize_angle(const double theta, const int theta_size)
{
  const double range = 2.0 * M_PI / theta_size;
  return static_cast<int>(std::rint(autoware_utils_math::normalize_radian(theta, 0.0) / range)) % theta_size;
}

IndexXYT pose2index(
  const nav_msgs::OccupancyGrid & costmap, const geometry_msgs::Pose & pose_local,
  const int theta_size)
{
  return {
    static_cast<int>(pose_local.position.x / costmap.info.resolution),
    static_cast<int>(pose_local.position.y / costmap.info.resolution),
    discretize_angle(tf2::getYaw(pose_local.orientation), theta_size)};
}

geometry_msgs::Pose index2pose(
  const nav_msgs::OccupancyGrid & costmap, const IndexXYT & index, const int theta_size)
{
  geometry_msgs::Pose pose;
  pose.position.x = index.x * costmap.info.resolution;
  pose.position.y = index.y * costmap.info.resolution;
  pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(
    index.theta * (2.0 * M_PI / theta_size));
  return pose;
}

geometry_msgs::Pose global2local(
  const nav_msgs::OccupancyGrid & costmap, const geometry_msgs::Pose & pose_global)
{
  tf2::Transform origin;
  tf2::fromMsg(costmap.info.origin, origin);
  geometry_msgs::TransformStamped transform;
  transform.transform = tf2::toMsg(origin.inverse());
  return autoware_utils_geometry::transform_pose(pose_global, transform);
}

void OccupancyGridBasedCollisionDetector::setMap(const nav_msgs::OccupancyGrid & costmap)
{
  costmap_ = costmap;
  obstacle_table_.assign(costmap_.info.height, std::vector<bool>(costmap_.info.width, false));
  for (uint32_t y = 0; y < costmap_.info.height; ++y) {
    for (uint32_t x = 0; x < costmap_.info.width; ++x) {
      const int cost = costmap_.data[y * costmap_.info.width + x];
      obstacle_table_[y][x] = cost < 0 || param_.obstacle_threshold <= cost;
    }
  }
  collision_indexes_table_.clear();
  for (int theta = 0; theta < param_.theta_size; ++theta) {
    std::vector<IndexXY> indexes;
    computeCollisionIndexes(theta, &indexes);
    collision_indexes_table_.push_back(indexes);
  }
}

void OccupancyGridBasedCollisionDetector::computeCollisionIndexes(
  const int theta_index, std::vector<IndexXY> * indexes)
{
  const auto base_pose = index2pose(costmap_, {0, 0, theta_index}, param_.theta_size);
  const double theta = tf2::getYaw(base_pose.orientation);
  const auto add = [&](const double x, const double y) {
    geometry_msgs::Point point;
    point.x = base_pose.position.x + std::cos(theta) * x - std::sin(theta) * y;
    point.y = base_pose.position.y + std::sin(theta) * x + std::cos(theta) * y;
    indexes->push_back({
      static_cast<int>(point.x / costmap_.info.resolution),
      static_cast<int>(point.y / costmap_.info.resolution)});
  };
  const double back = -param_.vehicle_shape.base2back;
  const double front = param_.vehicle_shape.length - param_.vehicle_shape.base2back;
  const double right = -param_.vehicle_shape.width / 2.0;
  const double left = param_.vehicle_shape.width / 2.0;
  for (double x = back; x <= front; x += costmap_.info.resolution / 2.0) {
    for (double y = right; y <= left; y += costmap_.info.resolution / 2.0) add(x, y);
    add(x, left);
  }
  for (double y = right; y <= left; y += costmap_.info.resolution / 2.0) add(front, y);
  add(front, left);
}

bool OccupancyGridBasedCollisionDetector::isOutOfRange(const IndexXYT & index) const
{
  return index.x < 0 || index.y < 0 || static_cast<int>(costmap_.info.width) <= index.x ||
         static_cast<int>(costmap_.info.height) <= index.y;
}

bool OccupancyGridBasedCollisionDetector::isObstacle(const IndexXYT & index) const
{
  return obstacle_table_[index.y][index.x];
}

bool OccupancyGridBasedCollisionDetector::detectCollision(
  const IndexXYT & base_index, const bool check_out_of_range) const
{
  if (collision_indexes_table_.empty()) {
    std::cerr << "[occupancy_grid_based_collision_detector] setMap has not yet been done." << std::endl;
    return false;
  }
  for (const auto & offset : collision_indexes_table_.at(base_index.theta)) {
    const IndexXYT index{base_index.x + offset.x, base_index.y + offset.y, 0};
    if (check_out_of_range) {
      if (isOutOfRange(index) || isObstacle(index)) return true;
    } else {
      if (isOutOfRange(index)) return false;
      if (isObstacle(index)) return true;
    }
  }
  return false;
}

bool OccupancyGridBasedCollisionDetector::hasObstacleOnPath(
  const autoware_internal_planning_msgs::PathWithLaneId & path,
  const bool check_out_of_range) const
{
  for (const auto & point : path.points) {
    const auto local_pose = global2local(costmap_, point.point.pose);
    if (detectCollision(pose2index(costmap_, local_pose, param_.theta_size), check_out_of_range)) return true;
  }
  return false;
}
}  // namespace autoware::behavior_path_planner
