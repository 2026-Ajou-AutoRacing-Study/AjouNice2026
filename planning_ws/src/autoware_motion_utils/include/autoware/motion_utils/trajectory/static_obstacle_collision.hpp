#ifndef AUTOWARE_MOTION_UTILS_STATIC_OBSTACLE_COLLISION_HPP_
#define AUTOWARE_MOTION_UTILS_STATIC_OBSTACLE_COLLISION_HPP_

#include <autoware_planning_msgs/StaticObstacle.h>
#include <autoware_planning_msgs/TrajectoryPoint.h>
#include <boost/geometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <vector>

namespace autoware::motion_utils
{
// Validate the swept rectangular footprint, including inter-sample rotations. Inflate by
// the maximum corner displacement between samples so a thin obstacle cannot fall in a gap.
template <typename VehicleInfo>
inline std::optional<size_t> firstObstacleCollision(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points, size_t start,
  const std::vector<autoware_planning_msgs::StaticObstacle> & obstacles,
  const VehicleInfo & vehicle)
{
  namespace bg = boost::geometry;
  using Point = bg::model::d2::point_xy<double>;
  using Polygon = bg::model::polygon<Point>;
  if (obstacles.empty() || points.empty()) return std::nullopt;
  const double front = vehicle.wheel_base_m + vehicle.front_overhang_m;
  const double rear = vehicle.rear_overhang_m;
  const double half_width = vehicle.vehicle_width_m / 2.0;
  const double corner_radius = std::hypot(std::max(front, rear), half_width);
  std::vector<Polygon> polygons;
  std::vector<std::array<double,4>> boxes;
  for (const auto & obstacle : obstacles) {
    Polygon poly;
    for (const auto & p : obstacle.polygon.points) poly.outer().emplace_back(p.x, p.y);
    bg::correct(poly);
    if (!bg::is_valid(poly) || !std::isfinite(obstacle.hard_margin) ||
        obstacle.hard_margin < 0.0) return start;
    bg::model::box<Point> box;
    bg::envelope(poly,box);
    const double padding=corner_radius+obstacle.hard_margin;
    boxes.push_back({box.min_corner().x()-padding,box.min_corner().y()-padding,
                     box.max_corner().x()+padding,box.max_corner().y()+padding});
    polygons.push_back(poly);
  }
  for (size_t i = start; i < points.size(); ++i) {
    const auto & a = points.at(i).pose;
    const auto & b = points.at(std::min(i + 1, points.size() - 1)).pose;
    std::vector<size_t> nearby;
    for (size_t o=0; o<boxes.size(); ++o) {
      const auto & box=boxes[o];
      if (std::max(a.position.x,b.position.x)<box[0] || std::min(a.position.x,b.position.x)>box[2] ||
          std::max(a.position.y,b.position.y)<box[1] || std::min(a.position.y,b.position.y)>box[3]) continue;
      nearby.push_back(o);
    }
    if (nearby.empty()) continue;
    const double yaw = tf2::getYaw(a.orientation);
    const double delta_yaw = std::atan2(std::sin(tf2::getYaw(b.orientation) - yaw),
                                      std::cos(tf2::getYaw(b.orientation) - yaw));
    const double travel = std::hypot(b.position.x - a.position.x, b.position.y - a.position.y);
    if (!std::isfinite(travel) || !std::isfinite(yaw)) return i;
    const int count = std::max(1, static_cast<int>(std::ceil((travel + corner_radius * std::abs(delta_yaw)) / 0.1)));
    const double sweep_buffer = (travel + corner_radius * std::abs(delta_yaw)) / (2.0 * count);
    for (int j = 0; j <= count; ++j) {
      const double f = static_cast<double>(j) / count;
      const double angle = yaw + f * delta_yaw;
      const double x = a.position.x + f * (b.position.x - a.position.x);
      const double y = a.position.y + f * (b.position.y - a.position.y);
      Polygon footprint;
      for (const auto & local : std::vector<Point>{{front,half_width},{front,-half_width},
                                                   {-rear,-half_width},{-rear,half_width}}) {
        footprint.outer().emplace_back(x + std::cos(angle)*local.x() - std::sin(angle)*local.y(),
                                       y + std::sin(angle)*local.x() + std::cos(angle)*local.y());
      }
      bg::correct(footprint);
      for (const size_t o : nearby) {
        if (bg::distance(footprint, polygons[o]) <= obstacles[o].hard_margin + sweep_buffer) return i;
      }
    }
  }
  return std::nullopt;
}
}  // namespace autoware::motion_utils
#endif
