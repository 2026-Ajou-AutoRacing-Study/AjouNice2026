// Copyright 2026 AjouNice2026
// Licensed under the Apache License, Version 2.0.
#ifndef AUTOWARE_MOTION_UTILS_STATIC_OBSTACLE_ESCAPE_HPP_
#define AUTOWARE_MOTION_UTILS_STATIC_OBSTACLE_ESCAPE_HPP_
#include <autoware/motion_utils/trajectory/static_obstacle_collision.hpp>
#include <limits>

namespace autoware::motion_utils
{
namespace obstacle_escape_detail
{
using Point = boost::geometry::model::d2::point_xy<double>;
using Polygon = boost::geometry::model::polygon<Point>;
template <typename VehicleInfo>
Polygon footprint(const geometry_msgs::Pose & pose, const VehicleInfo & vehicle)
{
  Polygon polygon;
  const double yaw = tf2::getYaw(pose.orientation);
  const double front = vehicle.wheel_base_m + vehicle.front_overhang_m;
  for (const auto & xy : std::array<std::array<double, 2>, 4>{{
      {front, vehicle.vehicle_width_m / 2.0}, {front, -vehicle.vehicle_width_m / 2.0},
      {-vehicle.rear_overhang_m, -vehicle.vehicle_width_m / 2.0},
      {-vehicle.rear_overhang_m, vehicle.vehicle_width_m / 2.0}}}) {
    polygon.outer().emplace_back(
      pose.position.x + std::cos(yaw) * xy[0] - std::sin(yaw) * xy[1],
      pose.position.y + std::sin(yaw) * xy[0] + std::cos(yaw) * xy[1]);
  }
  boost::geometry::correct(polygon);
  return polygon;
}
inline Polygon polygon(const autoware_planning_msgs::StaticObstacle & obstacle)
{
  Polygon p;
  for (const auto & v : obstacle.polygon.points) p.outer().emplace_back(v.x, v.y);
  boost::geometry::correct(p);
  return p;
}
inline double area(const Polygon & a, const Polygon & b)
{
  if (boost::geometry::disjoint(a, b)) return 0.0;
  std::vector<Polygon> parts;
  boost::geometry::intersection(a, b, parts);
  double result = 0.0;
  for (const auto & p : parts) result += std::abs(boost::geometry::area(p));
  return result;
}
}

// Only the measured initial state determines which obstacle constraints may use
// slack. No exception is granted to another obstacle later on the trajectory.
template <typename VehicleInfo>
std::optional<std::vector<bool>> initialObstacleOverlaps(
  const geometry_msgs::Pose & measured,
  const std::vector<autoware_planning_msgs::StaticObstacle> & obstacles,
  const VehicleInfo & vehicle)
{
  if (!std::isfinite(measured.position.x) || !std::isfinite(measured.position.y) ||
      !std::isfinite(tf2::getYaw(measured.orientation))) return {};
  const auto body = obstacle_escape_detail::footprint(measured, vehicle);
  std::vector<bool> result;
  for (const auto & obstacle : obstacles) {
    const auto p = obstacle_escape_detail::polygon(obstacle);
    if (!boost::geometry::is_valid(p) || !std::isfinite(obstacle.hard_margin) ||
        obstacle.hard_margin < 0.0) return {};
    result.push_back(boost::geometry::distance(body, p) <= obstacle.hard_margin);
  }
  return result;
}
struct StaticObstacleEscapeResult
{
  bool valid{false};
  bool needed{false};
  double contact_cost{0.0};
};

template <typename VehicleInfo>
StaticObstacleEscapeResult checkStaticObstacleEscape(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points,
  const geometry_msgs::Pose & measured,
  const std::vector<autoware_planning_msgs::StaticObstacle> & obstacles,
  const VehicleInfo & vehicle)
{
  namespace bg = boost::geometry;
  using namespace obstacle_escape_detail;
  StaticObstacleEscapeResult result;
  if (points.size() < 2) return result;
  const auto overlaps = initialObstacleOverlaps(measured, obstacles, vehicle);
  if (!overlaps) return result;
  result.needed = std::any_of(overlaps->begin(), overlaps->end(), [](bool b) { return b; });
  if (!result.needed) return result;
  size_t start = 0;
  double nearest = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < points.size(); ++i) {
    const auto & p = points[i].pose;
    if (!std::isfinite(p.position.x) || !std::isfinite(p.position.y) ||
        !std::isfinite(tf2::getYaw(p.orientation))) return result;
    const double d = std::hypot(p.position.x-measured.position.x, p.position.y-measured.position.y);
    if (d < nearest) { nearest = d; start = i; }
  }
  // Same measured-state continuity contract as PO's ordinary candidates.
  const double yaw_gap = tf2::getYaw(points[start].pose.orientation)-tf2::getYaw(measured.orientation);
  if (nearest > 1.0 || std::abs(std::atan2(std::sin(yaw_gap), std::cos(yaw_gap))) > 0.35) return result;
  std::vector<autoware_planning_msgs::StaticObstacle> ordinary;
  std::vector<Polygon> contacts;
  std::vector<double> margins;
  const auto initial = footprint(measured, vehicle);
  double initial_area = 0.0, initial_deficit = 0.0;
  for (size_t i = 0; i < obstacles.size(); ++i) {
    if (!overlaps->at(i)) { ordinary.push_back(obstacles[i]); continue; }
    contacts.push_back(polygon(obstacles[i])); margins.push_back(obstacles[i].hard_margin);
    initial_area += area(initial, contacts.back());
    initial_deficit += std::max(0.0, margins.back()-bg::distance(initial, contacts.back()));
  }
  if (firstObstacleCollision(points, start, ordinary, vehicle)) return result;
  const double corner_radius = std::hypot(
    std::max(vehicle.wheel_base_m+vehicle.front_overhang_m, vehicle.rear_overhang_m),
    vehicle.vehicle_width_m/2.0);
  // Broad phase only: distant segments contribute exactly zero contact area.
  // Do not build/intersect a footprint for the entire extended route suffix.
  std::vector<std::array<double,4>> boxes;
  for (const auto & contact : contacts) {
    bg::model::box<Point> box; bg::envelope(contact,box);
    boxes.push_back({box.min_corner().x()-corner_radius,box.min_corner().y()-corner_radius,
      box.max_corner().x()+corner_radius,box.max_corner().y()+corner_radius});
  }
  for (size_t i = start; i+1 < points.size(); ++i) {
    const auto & a = points[i].pose; const auto & b = points[i+1].pose;
    std::vector<size_t> nearby;
    for (size_t o=0;o<boxes.size();++o) {
      const auto & box=boxes[o];
      if (std::max(a.position.x,b.position.x)<box[0] || std::min(a.position.x,b.position.x)>box[2] ||
          std::max(a.position.y,b.position.y)<box[1] || std::min(a.position.y,b.position.y)>box[3]) continue;
      nearby.push_back(o);
    }
    if (nearby.empty()) continue;
    const double yaw = tf2::getYaw(a.orientation), angle = tf2::getYaw(b.orientation)-yaw;
    const double delta = std::atan2(std::sin(angle), std::cos(angle));
    const double travel = std::hypot(b.position.x-a.position.x, b.position.y-a.position.y);
    const int count = std::max(1, static_cast<int>(std::ceil((travel+corner_radius*std::abs(delta))/0.1)));
    for (int j = 0; j <= count; ++j) {
      const double f = static_cast<double>(j)/count;
      auto pose = a; pose.position.x += f*(b.position.x-a.position.x);
      pose.position.y += f*(b.position.y-a.position.y);
      pose.orientation.x = pose.orientation.y = 0.0;
      pose.orientation.z = std::sin((yaw+f*delta)/2.0);
      pose.orientation.w = std::cos((yaw+f*delta)/2.0);
      const auto body = footprint(pose, vehicle);
      for (const size_t o : nearby) result.contact_cost += area(body, contacts[o])*travel/count;
    }
  }
  const auto terminal = footprint(points.back().pose, vehicle);
  double terminal_area = 0.0, terminal_deficit = 0.0;
  for (size_t i = 0; i < contacts.size(); ++i) {
    terminal_area += area(terminal, contacts[i]);
    terminal_deficit += std::max(0.0, margins[i]-bg::distance(terminal, contacts[i]));
  }
  // Prefer escaping over continued penetration, without a fixed contact-area
  // allowance or a new speed cap. Collision cost ranks admissible alternatives.
  result.valid = terminal_area < initial_area ||
    (terminal_area <= initial_area && terminal_deficit < initial_deficit);
  return result;
}
}
#endif
