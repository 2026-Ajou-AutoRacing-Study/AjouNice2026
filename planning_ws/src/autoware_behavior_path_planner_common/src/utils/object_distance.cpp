#include <autoware/behavior_path_planner_common/utils/object_distance.hpp>

#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_geometry/pose_deviation.hpp>

#include <boost/geometry/algorithms/intersects.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
using Point2d = autoware_utils_geometry::Point2d;

double pointToSegmentDistance(const Point2d & point, const Point2d & first, const Point2d & second)
{
  const double dx = second.x() - first.x();
  const double dy = second.y() - first.y();
  const double length_squared = dx * dx + dy * dy;
  if (length_squared == 0.0) return std::hypot(point.x() - first.x(), point.y() - first.y());
  const double projection = std::clamp(
    ((point.x() - first.x()) * dx + (point.y() - first.y()) * dy) / length_squared,
    0.0, 1.0);
  return std::hypot(
    point.x() - (first.x() + projection * dx), point.y() - (first.y() + projection * dy));
}

template <typename Points>
double pointToBoundaryDistance(const Point2d & point, const Points & boundary)
{
  if (boundary.size() < 2U) return std::numeric_limits<double>::max();
  double result = std::numeric_limits<double>::max();
  for (size_t index = 0; index < boundary.size(); ++index) {
    result = std::min(
      result, pointToSegmentDistance(point, boundary.at(index), boundary.at((index + 1U) % boundary.size())));
  }
  return result;
}

template <typename FirstPoints, typename SecondPoints>
double boundaryDistance(const FirstPoints & first, const SecondPoints & second)
{
  double result = std::numeric_limits<double>::max();
  for (const auto & point : first) result = std::min(result, pointToBoundaryDistance(point, second));
  for (const auto & point : second) result = std::min(result, pointToBoundaryDistance(point, first));
  return result;
}
}  // namespace

namespace autoware::behavior_path_planner::utils
{

double l2Norm(const geometry_msgs::Vector3 & vector)
{
  return std::sqrt(
    vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}

bool checkCollisionBetweenPathFootprintsAndObjects(
  const autoware_utils_geometry::LinearRing2d & local_vehicle_footprint,
  const autoware_internal_planning_msgs::PathWithLaneId & ego_path,
  const autoware_perception_msgs::PredictedObjects & dynamic_objects, const double margin)
{
  return std::any_of(
    ego_path.points.begin(), ego_path.points.end(), [&](const auto & point) {
      return checkCollisionBetweenFootprintAndObjects(
        local_vehicle_footprint, point.point.pose, dynamic_objects, margin);
    });
}

bool checkCollisionBetweenFootprintAndObjects(
  const autoware_utils_geometry::LinearRing2d & local_vehicle_footprint,
  const geometry_msgs::Pose & ego_pose,
  const autoware_perception_msgs::PredictedObjects & dynamic_objects, const double margin)
{
  const auto vehicle_footprint = autoware_utils_geometry::transform_vector(
    local_vehicle_footprint, autoware_utils_geometry::pose2transform(ego_pose));
  // This ROS1 Boost version has no Polygon2d distance strategy.  Intersecting
  // areas have distance zero; otherwise the Euclidean minimum over both closed
  // polygon boundaries is equivalent to the upstream boundary distance.
  autoware_utils_geometry::Polygon2d vehicle_polygon;
  vehicle_polygon.outer().assign(vehicle_footprint.begin(), vehicle_footprint.end());
  return std::any_of(dynamic_objects.objects.begin(), dynamic_objects.objects.end(),
    [&](const auto & object) {
      const auto object_polygon = autoware_utils_geometry::to_polygon2d(object);
      if (boost::geometry::intersects(object_polygon, vehicle_polygon)) return 0.0 < margin;
      return boundaryDistance(object_polygon.outer(), vehicle_polygon.outer()) < margin;
    });
}

double calcLateralDistanceFromEgoToObject(
  const geometry_msgs::Pose & ego_pose, const double vehicle_width,
  const autoware_perception_msgs::PredictedObject & dynamic_object)
{
  double min_distance = std::numeric_limits<double>::max();
  const auto object_polygon = autoware_utils_geometry::to_polygon2d(dynamic_object);
  const auto left_pose = autoware_utils_geometry::calc_offset_pose(
    ego_pose, 0.0, vehicle_width / 2.0, 0.0);
  const auto right_pose = autoware_utils_geometry::calc_offset_pose(
    ego_pose, 0.0, -vehicle_width / 2.0, 0.0);
  for (const auto & vertex : object_polygon.outer()) {
    const auto point = autoware_utils_geometry::create_point(vertex.x(), vertex.y(), 0.0);
    const auto from_left = autoware_utils_geometry::calc_lateral_deviation(left_pose, point);
    const auto from_right = autoware_utils_geometry::calc_lateral_deviation(right_pose, point);
    if (from_left < 0.0 && from_right > 0.0) return 0.0;
    min_distance = std::min(min_distance, std::min(std::abs(from_left), std::abs(from_right)));
  }
  return min_distance;
}

double calcLongitudinalDistanceFromEgoToObject(
  const geometry_msgs::Pose & ego_pose, const double base_link2front,
  const double base_link2rear, const autoware_perception_msgs::PredictedObject & dynamic_object)
{
  double min_distance = std::numeric_limits<double>::max();
  const auto object_polygon = autoware_utils_geometry::to_polygon2d(dynamic_object);
  const auto front_pose = autoware_utils_geometry::calc_offset_pose(
    ego_pose, base_link2front, 0.0, 0.0);
  const auto rear_pose = autoware_utils_geometry::calc_offset_pose(
    ego_pose, base_link2rear, 0.0, 0.0);
  for (const auto & vertex : object_polygon.outer()) {
    const auto point = autoware_utils_geometry::create_point(vertex.x(), vertex.y(), 0.0);
    const auto from_front = autoware_utils_geometry::calc_longitudinal_deviation(front_pose, point);
    const auto from_rear = -autoware_utils_geometry::calc_longitudinal_deviation(rear_pose, point);
    if (from_front < 0.0 && from_rear < 0.0) return 0.0;
    min_distance = std::min(min_distance, std::min(std::abs(from_front), std::abs(from_rear)));
  }
  return min_distance;
}

double calcLongitudinalDistanceFromEgoToObjects(
  const geometry_msgs::Pose & ego_pose, const double base_link2front, const double base_link2rear,
  const autoware_perception_msgs::PredictedObjects & dynamic_objects)
{
  double min_distance = std::numeric_limits<double>::max();
  for (const auto & object : dynamic_objects.objects) {
    min_distance = std::min(
      min_distance, calcLongitudinalDistanceFromEgoToObject(
                      ego_pose, base_link2front, base_link2rear, object));
  }
  return min_distance;
}

}  // namespace autoware::behavior_path_planner::utils
