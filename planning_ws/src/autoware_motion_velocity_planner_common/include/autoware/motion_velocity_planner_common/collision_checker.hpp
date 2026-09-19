#pragma once

#include <autoware_utils_geometry/boost_geometry.hpp>

#include <boost/geometry/algorithms/envelope.hpp>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <memory>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner
{
namespace bgi = boost::geometry::index;
using RtreeNode = std::pair<autoware_utils_geometry::Box2d, size_t>;
using Rtree = bgi::rtree<RtreeNode, bgi::rstar<16>>;
struct Collision
{
  size_t trajectory_index{};
  autoware_utils_geometry::MultiPoint2d collision_points;
};
class CollisionChecker
{
  const autoware_utils_geometry::MultiPolygon2d trajectory_footprints_;
  std::shared_ptr<Rtree> rtree_;
public:
  explicit CollisionChecker(autoware_utils_geometry::MultiPolygon2d trajectory_footprints);
  template <class Geometry> [[nodiscard]] std::vector<Collision> getCollisions(const Geometry & geometry) const;
  [[nodiscard]] std::shared_ptr<const Rtree> getRtree() const { return rtree_; }
  [[nodiscard]] size_t trajectorySize() const { return trajectory_footprints_.size(); }
};
}  // namespace autoware::motion_velocity_planner
