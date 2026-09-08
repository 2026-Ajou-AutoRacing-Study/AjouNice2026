#include <autoware/motion_velocity_planner_common/collision_checker.hpp>

#include <type_traits>

namespace autoware::motion_velocity_planner
{
CollisionChecker::CollisionChecker(autoware_utils_geometry::MultiPolygon2d trajectory_footprints)
: trajectory_footprints_(std::move(trajectory_footprints))
{
  std::vector<RtreeNode> nodes;
  nodes.reserve(trajectory_footprints_.size());
  for (size_t index = 0; index < trajectory_footprints_.size(); ++index) {
    nodes.emplace_back(boost::geometry::return_envelope<autoware_utils_geometry::Box2d>(trajectory_footprints_.at(index)), index);
  }
  rtree_ = std::make_shared<Rtree>(nodes.begin(), nodes.end());
}

template <class Geometry>
std::vector<Collision> CollisionChecker::getCollisions(const Geometry & geometry) const
{
  std::vector<Collision> collisions;
  std::vector<RtreeNode> candidates;
  rtree_->query(bgi::intersects(geometry), std::back_inserter(candidates));
  for (const auto & candidate : candidates) {
    autoware_utils_geometry::MultiPoint2d intersections;
    if constexpr (std::is_same_v<typename boost::geometry::tag<Geometry>::type, boost::geometry::point_tag>) {
      // Boost 1.71 cannot materialize polygon-point intersection output.  A point collision is
      // exactly covered_by and its concrete contact point is the query point itself.
      if (boost::geometry::covered_by(geometry, trajectory_footprints_.at(candidate.second))) {
        intersections.push_back(geometry);
      }
    } else {
      boost::geometry::intersection(trajectory_footprints_.at(candidate.second), geometry, intersections);
      // Boost 1.71 also produces no point output for areal intersections. Preserve the actual
      // intersection boundary so downstream modules receive a non-empty contact set.
      if constexpr (
        std::is_same_v<typename boost::geometry::tag<Geometry>::type, boost::geometry::polygon_tag> ||
        std::is_same_v<typename boost::geometry::tag<Geometry>::type, boost::geometry::multi_polygon_tag>) {
        if (intersections.empty()) {
          autoware_utils_geometry::MultiPolygon2d areal_intersections;
          boost::geometry::intersection(
            trajectory_footprints_.at(candidate.second), geometry, areal_intersections);
          for (const auto & area : areal_intersections) {
            intersections.insert(
              intersections.end(), area.outer().begin(), area.outer().end());
          }
        }
      }
    }
    if (!intersections.empty()) collisions.push_back({candidate.second, intersections});
  }
  return collisions;
}

template std::vector<Collision> CollisionChecker::getCollisions<autoware_utils_geometry::Point2d>(const autoware_utils_geometry::Point2d &) const;
template std::vector<Collision> CollisionChecker::getCollisions<autoware_utils_geometry::Line2d>(const autoware_utils_geometry::Line2d &) const;
template std::vector<Collision> CollisionChecker::getCollisions<autoware_utils_geometry::Polygon2d>(const autoware_utils_geometry::Polygon2d &) const;
template std::vector<Collision> CollisionChecker::getCollisions<autoware_utils_geometry::MultiPolygon2d>(const autoware_utils_geometry::MultiPolygon2d &) const;
}  // namespace autoware::motion_velocity_planner
