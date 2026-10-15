#ifndef AUTOWARE_PATH_OPTIMIZER_OBSTACLE_COLLISION_HPP_
#define AUTOWARE_PATH_OPTIMIZER_OBSTACLE_COLLISION_HPP_

#include "autoware/path_optimizer/common_structs.hpp"
#include <autoware/motion_utils/trajectory/static_obstacle_collision.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <boost/geometry.hpp>
#include <array>
#include <tf2/utils.h>

namespace autoware::path_optimizer
{
inline bool isAlignedWithEgoPose(
  const geometry_msgs::Pose & planned, const geometry_msgs::Pose & measured)
{
  const double gap = std::hypot(planned.position.x - measured.position.x,
                                planned.position.y - measured.position.y);
  const double yaw = tf2::getYaw(planned.orientation) - tf2::getYaw(measured.orientation);
  return gap <= 1.0 && std::abs(std::atan2(std::sin(yaw), std::cos(yaw))) <= 0.35;
}

inline std::optional<size_t> firstCurvatureViolation(
  const std::vector<TrajectoryPoint> & points, size_t start, size_t end, double limit)
{
  if (points.size() < 3) return std::nullopt;
  std::vector<double> arc(points.size(), 0.0);
  for (size_t i = 1; i < points.size(); ++i) {
    arc[i] = arc[i - 1] + std::hypot(points[i].pose.position.x - points[i - 1].pose.position.x,
                                  points[i].pose.position.y - points[i - 1].pose.position.y);
  }
  for (size_t i = std::max(start, size_t{1}); i < std::min(end, points.size() - 1); ++i) {
    // A metre-wide estimate can hide a short post-resampling kink. Check the
    // actual adjacent vertices as well: the vehicle must traverse those points.
    const auto & local_a = points[i - 1].pose.position;
    const auto & local_b = points[i].pose.position;
    const auto & local_c = points[i + 1].pose.position;
    const double local_denominator =
      std::hypot(local_b.x-local_a.x, local_b.y-local_a.y) *
      std::hypot(local_c.x-local_b.x, local_c.y-local_b.y) *
      std::hypot(local_c.x-local_a.x, local_c.y-local_a.y);
    if (local_denominator > 1e-8) {
      const double local_curvature = 2.0 * std::abs(
        (local_b.x-local_a.x)*(local_c.y-local_a.y) -
        (local_b.y-local_a.y)*(local_c.x-local_a.x)) / local_denominator;
      if (!std::isfinite(local_curvature) || local_curvature > limit + 1e-3) return i;
    }
    size_t lo = i - 1, hi = i + 1;
    while (lo > 0 && arc[i] - arc[lo] < 1.0) --lo;
    while (hi + 1 < points.size() && arc[hi] - arc[i] < 1.0) ++hi;
    const auto & a = points[lo].pose.position;
    const auto & b = points[i].pose.position;
    const auto & c = points[hi].pose.position;
    const double denominator = std::hypot(b.x-a.x,b.y-a.y) * std::hypot(c.x-b.x,c.y-b.y) *
      std::hypot(c.x-a.x,c.y-a.y);
    if (denominator < 1e-8) continue;
    const double curvature = 2.0 * std::abs((b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x)) / denominator;
    if (!std::isfinite(curvature) || curvature > limit + 1e-3) return i;
  }
  return std::nullopt;
}

inline bool measuredFootprintInsideRoad(
  const geometry_msgs::Pose & pose, const std::vector<geometry_msgs::Point> & left,
  const std::vector<geometry_msgs::Point> & right,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle, double margin)
{
  namespace bg = boost::geometry;
  using Point = bg::model::d2::point_xy<double>;
  using Polygon = bg::model::polygon<Point>;
  Polygon road, footprint;
  bg::model::linestring<Point> left_line, right_line;
  for (const auto & p : left) { road.outer().emplace_back(p.x,p.y); left_line.emplace_back(p.x,p.y); }
  for (auto it = right.rbegin(); it != right.rend(); ++it) road.outer().emplace_back(it->x,it->y);
  for (const auto & p : right) right_line.emplace_back(p.x,p.y);
  bg::correct(road);
  if (left.size() < 2 || right.size() < 2 || !bg::is_valid(road)) return false;
  const double yaw = tf2::getYaw(pose.orientation), c = std::cos(yaw), s = std::sin(yaw);
  const double front = vehicle.wheel_base_m + vehicle.front_overhang_m;
  const double rear = vehicle.rear_overhang_m, half = vehicle.vehicle_width_m / 2.0;
  for (const auto & p : std::vector<Point>{{front,half},{front,-half},{-rear,-half},{-rear,half}}) {
    footprint.outer().emplace_back(pose.position.x+c*p.x()-s*p.y(), pose.position.y+s*p.x()+c*p.y());
  }
  bg::correct(footprint);
  return bg::covered_by(footprint,road) && bg::distance(footprint,left_line) >= margin &&
    bg::distance(footprint,right_line) >= margin;
}

inline std::vector<double> obstacleCoveringRadii(
  const autoware::vehicle_info_utils::VehicleInfo & vehicle,
  const std::vector<double> & offsets)
{
  std::vector<double> radii;
  for (size_t i = 0; i < offsets.size(); ++i) {
    const double back = i == 0 ? -vehicle.rear_overhang_m : (offsets[i - 1] + offsets[i]) / 2.0;
    const double front = i + 1 == offsets.size() ? vehicle.wheel_base_m + vehicle.front_overhang_m :
      (offsets[i] + offsets[i + 1]) / 2.0;
    const double longitudinal = std::max(std::abs(offsets[i] - back), std::abs(front - offsets[i]));
    radii.push_back(std::hypot(vehicle.vehicle_width_m / 2.0, longitudinal));
  }
  return radii;
}

inline double obstacleSoftViolation(
  const geometry_msgs::Pose & pose,
  const std::vector<autoware_planning_msgs::StaticObstacle> & obstacles,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle)
{
  namespace bg = boost::geometry;
  using Point = bg::model::d2::point_xy<double>;
  using Polygon = bg::model::polygon<Point>;
  const double yaw = tf2::getYaw(pose.orientation), c = std::cos(yaw), s = std::sin(yaw);
  const double front = vehicle.wheel_base_m + vehicle.front_overhang_m;
  const double rear = vehicle.rear_overhang_m, half = vehicle.vehicle_width_m / 2.0;
  Polygon footprint;
  for (const auto & p : std::vector<Point>{{front,half},{front,-half},{-rear,-half},{-rear,half}}) {
    footprint.outer().emplace_back(pose.position.x+c*p.x()-s*p.y(), pose.position.y+s*p.x()+c*p.y());
  }
  bg::correct(footprint);
  double violation = 0.0;
  for (const auto & obstacle : obstacles) {
    double min_x=1e10, min_y=1e10, max_x=-1e10, max_y=-1e10;
    for (const auto & p : obstacle.polygon.points) {
      min_x=std::min<double>(min_x,p.x); min_y=std::min<double>(min_y,p.y);
      max_x=std::max<double>(max_x,p.x); max_y=std::max<double>(max_y,p.y);
    }
    const double dx=std::max({min_x-pose.position.x,0.0,pose.position.x-max_x});
    const double dy=std::max({min_y-pose.position.y,0.0,pose.position.y-max_y});
    if (std::hypot(dx,dy) > std::hypot(std::max(front,rear),half) +
        obstacle.hard_margin + obstacle.soft_margin) continue;
    Polygon polygon;
    for (const auto & p : obstacle.polygon.points) polygon.outer().emplace_back(p.x,p.y);
    bg::correct(polygon);
    violation += std::max(0.0, obstacle.hard_margin + obstacle.soft_margin - bg::distance(footprint,polygon));
  }
  return violation;
}

using autoware::motion_utils::firstObstacleCollision;
}  // namespace autoware::path_optimizer
#endif
