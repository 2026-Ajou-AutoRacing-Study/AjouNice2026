#include "body_longitudinal_filter.hpp"
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <boost/geometry.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace autoware::motion_velocity_planner
{
std::optional<BodyLongitudinalBounds> body_longitudinal_bounds(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory,
  const geometry_msgs::Pose & ego_pose,
  const autoware_perception_msgs::PredictedObject & object,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle)
{
  using namespace autoware_utils_geometry;
  if (trajectory.size() < 2) return std::nullopt;
  for (const auto & p : trajectory) {
    if (!std::isfinite(p.pose.position.x) || !std::isfinite(p.pose.position.y)) {
      return std::nullopt;
    }
  }
  if (autoware::motion_utils::calcSignedArcLength(trajectory, size_t{0}, trajectory.size()-1) <
      1e-6) return std::nullopt;
  const auto object_polygon = to_polygon2d(object);
  Polygon2d ego_polygon;
  for (const auto & offset : std::vector<std::pair<double, double>>{
         {vehicle.max_longitudinal_offset_m, vehicle.vehicle_width_m / 2.0},
         {vehicle.max_longitudinal_offset_m, -vehicle.vehicle_width_m / 2.0},
         {-vehicle.rear_overhang_m, -vehicle.vehicle_width_m / 2.0},
         {-vehicle.rear_overhang_m, vehicle.vehicle_width_m / 2.0}}) {
    const auto p = calc_offset_pose(ego_pose, offset.first, offset.second, 0.0).position;
    ego_polygon.outer().emplace_back(p.x, p.y);
  }
  boost::geometry::correct(ego_polygon);
  auto range = [&](const Polygon2d & polygon) -> std::optional<std::pair<double, double>> {
    if (polygon.outer().size() < 4) return std::nullopt;
    double lo = std::numeric_limits<double>::infinity();
    double hi = -lo;
    for (const auto & vertex : polygon.outer()) {
      if (!std::isfinite(vertex.x()) || !std::isfinite(vertex.y())) return std::nullopt;
      geometry_msgs::Point point;
      point.x = vertex.x();
      point.y = vertex.y();
      const double s = autoware::motion_utils::calcSignedArcLength(
        trajectory, ego_pose.position, point);
      if (!std::isfinite(s)) return std::nullopt;
      lo = std::min(lo, s);
      hi = std::max(hi, s);
    }
    if (!boost::geometry::is_valid(polygon) || std::abs(boost::geometry::area(polygon)) < 1e-6) {
      return std::nullopt;
    }
    return std::make_pair(lo, hi);
  };
  const auto object_range = range(object_polygon);
  const auto ego_range = range(ego_polygon);
  if (!object_range || !ego_range) return std::nullopt;
  return BodyLongitudinalBounds{
    object_range->second, ego_range->first,
    !boost::geometry::disjoint(object_polygon, ego_polygon)};
}
}  // namespace autoware::motion_velocity_planner
