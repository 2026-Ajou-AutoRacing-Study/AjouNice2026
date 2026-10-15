// Copyright 2026 Ajou University

#include "autoware/path_optimizer/obstacle_clustering.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/multi_point.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <tuple>
#include <utility>
#include <vector>

namespace autoware::path_optimizer
{
namespace
{
namespace bg = boost::geometry;
using BoostPoint = bg::model::d2::point_xy<double>;
using BoostPolygon = bg::model::polygon<BoostPoint>;

struct ObstacleGeometry
{
  BoostPolygon polygon;
  geometry_msgs::Point center;
  size_t reference_index{0};
  double lateral{0.0};
  bool valid{false};
};

size_t nearestReferenceIndex(
  const std::vector<TrajectoryPoint> & reference, const geometry_msgs::Point & point)
{
  size_t nearest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < reference.size(); ++i) {
    const double dx = point.x - reference[i].pose.position.x;
    const double dy = point.y - reference[i].pose.position.y;
    const double squared = dx * dx + dy * dy;
    if (squared < best) {
      best = squared;
      nearest = i;
    }
  }
  return nearest;
}

ObstacleGeometry makeGeometry(
  const autoware_planning_msgs::StaticObstacle & obstacle,
  const std::vector<TrajectoryPoint> & reference)
{
  ObstacleGeometry geometry;
  if (obstacle.polygon.points.size() < 3 || reference.empty()) return geometry;

  for (const auto & point : obstacle.polygon.points) {
    geometry.polygon.outer().emplace_back(point.x, point.y);
    geometry.center.x += point.x;
    geometry.center.y += point.y;
  }
  geometry.center.x /= obstacle.polygon.points.size();
  geometry.center.y /= obstacle.polygon.points.size();
  bg::correct(geometry.polygon);
  if (!bg::is_valid(geometry.polygon)) return geometry;

  geometry.reference_index = nearestReferenceIndex(reference, geometry.center);
  const auto & pose = reference[geometry.reference_index].pose;
  const double yaw = tf2::getYaw(pose.orientation);
  geometry.lateral =
    -std::sin(yaw) * (geometry.center.x - pose.position.x) +
    std::cos(yaw) * (geometry.center.y - pose.position.y);
  geometry.valid = true;
  return geometry;
}

bool sameSide(const ObstacleGeometry & lhs, const ObstacleGeometry & rhs)
{
  constexpr double center_tolerance = 0.25;
  if (std::abs(lhs.lateral) <= center_tolerance || std::abs(rhs.lateral) <= center_tolerance) {
    return true;
  }
  return std::signbit(lhs.lateral) == std::signbit(rhs.lateral);
}

bool extentFits(
  const std::vector<size_t> & members, const size_t added,
  const std::vector<autoware_planning_msgs::StaticObstacle> & obstacles,
  const std::vector<TrajectoryPoint> & reference,
  const ObstacleClusteringParameters & parameters)
{
  geometry_msgs::Point center;
  size_t point_count = 0;
  const auto accumulate = [&](const size_t index) {
    for (const auto & point : obstacles[index].polygon.points) {
      center.x += point.x;
      center.y += point.y;
      ++point_count;
    }
  };
  for (const auto member : members) accumulate(member);
  accumulate(added);
  if (point_count == 0) return false;
  center.x /= point_count;
  center.y /= point_count;

  const auto & pose = reference[nearestReferenceIndex(reference, center)].pose;
  const double yaw = tf2::getYaw(pose.orientation);
  const double c = std::cos(yaw), s = std::sin(yaw);
  double min_longitudinal = std::numeric_limits<double>::infinity();
  double max_longitudinal = -std::numeric_limits<double>::infinity();
  double min_lateral = std::numeric_limits<double>::infinity();
  double max_lateral = -std::numeric_limits<double>::infinity();
  const auto project = [&](const size_t index) {
    for (const auto & point : obstacles[index].polygon.points) {
      const double dx = point.x - pose.position.x;
      const double dy = point.y - pose.position.y;
      const double longitudinal = c * dx + s * dy;
      const double lateral = -s * dx + c * dy;
      min_longitudinal = std::min(min_longitudinal, longitudinal);
      max_longitudinal = std::max(max_longitudinal, longitudinal);
      min_lateral = std::min(min_lateral, lateral);
      max_lateral = std::max(max_lateral, lateral);
    }
  };
  for (const auto member : members) project(member);
  project(added);
  return max_longitudinal - min_longitudinal <= parameters.max_longitudinal_span &&
         max_lateral - min_lateral <= parameters.max_lateral_span;
}

autoware_planning_msgs::StaticObstacle mergeGroup(
  const std::vector<size_t> & members,
  const std::vector<autoware_planning_msgs::StaticObstacle> & obstacles)
{
  if (members.size() == 1) return obstacles[members.front()];

  auto merged = obstacles[members.front()];
  bg::model::multi_point<BoostPoint> vertices;
  for (const auto member : members) {
    merged.hard_margin = std::max(merged.hard_margin, obstacles[member].hard_margin);
    merged.soft_margin = std::max(merged.soft_margin, obstacles[member].soft_margin);
    for (const auto & point : obstacles[member].polygon.points) {
      vertices.emplace_back(point.x, point.y);
    }
  }
  BoostPolygon hull;
  bg::convex_hull(vertices, hull);
  if (hull.outer().size() < 4) return merged;

  merged.polygon.points.clear();
  // Boost closes the ring by repeating the first point; ROS polygons do not need it.
  for (size_t i = 0; i + 1 < hull.outer().size(); ++i) {
    geometry_msgs::Point32 point;
    point.x = hull.outer()[i].x();
    point.y = hull.outer()[i].y();
    merged.polygon.points.push_back(point);
  }
  return merged;
}
}  // namespace

std::vector<autoware_planning_msgs::StaticObstacle> clusterStaticObstacles(
  const std::vector<autoware_planning_msgs::StaticObstacle> & obstacles,
  const std::vector<TrajectoryPoint> & reference,
  const ObstacleClusteringParameters & parameters)
{
  if (obstacles.size() < parameters.trigger_count || reference.empty()) return obstacles;

  std::vector<ObstacleGeometry> geometries;
  geometries.reserve(obstacles.size());
  for (const auto & obstacle : obstacles) geometries.push_back(makeGeometry(obstacle, reference));

  std::vector<size_t> order(obstacles.size());
  std::iota(order.begin(), order.end(), size_t{0});
  std::stable_sort(order.begin(), order.end(), [&](const size_t lhs, const size_t rhs) {
    return std::tie(geometries[lhs].reference_index, geometries[lhs].lateral) <
           std::tie(geometries[rhs].reference_index, geometries[rhs].lateral);
  });

  std::vector<std::vector<size_t>> groups;
  for (const auto index : order) {
    bool inserted = false;
    if (geometries[index].valid) {
      for (auto & group : groups) {
        const bool close_to_member = std::any_of(group.begin(), group.end(), [&](const size_t member) {
          return geometries[member].valid && sameSide(geometries[index], geometries[member]) &&
                 bg::distance(geometries[index].polygon, geometries[member].polygon) <=
                   parameters.polygon_gap_threshold;
        });
        if (close_to_member && extentFits(group, index, obstacles, reference, parameters)) {
          group.push_back(index);
          inserted = true;
          break;
        }
      }
    }
    if (!inserted) groups.push_back({index});
  }

  std::vector<autoware_planning_msgs::StaticObstacle> clustered;
  clustered.reserve(groups.size());
  for (const auto & group : groups) clustered.push_back(mergeGroup(group, obstacles));
  return clustered;
}
}  // namespace autoware::path_optimizer
