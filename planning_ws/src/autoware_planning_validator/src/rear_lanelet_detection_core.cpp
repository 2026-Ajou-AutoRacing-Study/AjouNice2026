#include "autoware/planning_validator/rear_lanelet_detection_core.hpp"

#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>

#include <boost/geometry.hpp>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/LaneletSequence.h>

namespace autoware::planning_validator
{
lanelet::ConstLanelet make_rear_half_lanelet(
  const lanelet::ConstLanelet & lanelet, const bool is_right,
  const double ignore_width_from_centerline, const double expand_width_from_bound)
{
  lanelet::Points3d lefts;
  lanelet::Points3d rights;
  const double offset = is_right ? -ignore_width_from_centerline : ignore_width_from_centerline;
  const auto offset_centerline =
    autoware::experimental::lanelet2_utils::get_centerline_with_offset(lanelet, offset);
  const auto original_left_bound = !is_right
    ? autoware::experimental::lanelet2_utils::get_left_bound_with_offset(
        lanelet, expand_width_from_bound)
    : offset_centerline;
  const auto original_right_bound = !is_right
    ? offset_centerline
    : autoware::experimental::lanelet2_utils::get_right_bound_with_offset(
        lanelet, expand_width_from_bound);
  for (const auto & point : original_left_bound) lefts.emplace_back(point);
  for (const auto & point : original_right_bound) rights.emplace_back(point);
  return lanelet::Lanelet(
    lanelet::InvalId, lanelet::LineString3d(lanelet::InvalId, std::move(lefts)),
    lanelet::LineString3d(lanelet::InvalId, std::move(rights)));
}

lanelet::BasicPolygon3d make_rear_detection_polygon(
  const lanelet::ConstLanelets & lanelets, const geometry_msgs::Pose & ego_pose,
  const double forward_distance, const double backward_distance)
{
  const double ego_arc =
    autoware::experimental::lanelet2_utils::get_arc_coordinates(lanelets, ego_pose).length;
  const auto polygon = autoware::experimental::lanelet2_utils::get_polygon_from_arc_length(
    lanelets, ego_arc - backward_distance, ego_arc + forward_distance);
  return polygon.has_value() ? polygon->basicPolygon() : lanelet::BasicPolygon3d();
}

void cut_rear_detection_areas_by_current_lanes(
  const lanelet::ConstLanelets & current_lanes, std::vector<RearDetectionArea> & areas)
{
  const auto combined =
    autoware::experimental::lanelet2_utils::combine_lanelets_shape(current_lanes);
  if (!combined.has_value()) return;

  autoware_utils_geometry::Polygon2d current_polygon;
  boost::geometry::convert(combined->polygon2d().basicPolygon(), current_polygon);
  for (auto & area : areas) {
    if (area.polygon.empty()) continue;
    autoware_utils_geometry::Polygon2d original;
    boost::geometry::convert(lanelet::utils::to2D(area.polygon), original);
    autoware_utils_geometry::MultiPolygon2d difference;
    boost::geometry::difference(original, current_polygon, difference);
    if (difference.empty()) continue;
    lanelet::BasicPolygon3d result;
    for (const auto & point : difference.front().outer()) {
      result.push_back(lanelet::BasicPoint3d(point.x(), point.y(), area.polygon.front().z()));
    }
    area.polygon = result;
  }
}

std::vector<RearDetectionArea> make_rear_previous_detection_areas(
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & target_lanes,
  const double s1, const double s2,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const double left_offset, const double right_offset)
{
  return make_rear_previous_detection_areas(
    current_lanes, target_lanes, s1, s2, route_handler.get(), left_offset, right_offset);
}

std::vector<RearDetectionArea> make_rear_previous_detection_areas(
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & target_lanes,
  const double s1, const double s2,
  const autoware::route_handler::RouteHandler * route_handler,
  const double left_offset, const double right_offset)
{
  std::vector<RearDetectionArea> result;
  if (target_lanes.empty() || !route_handler) return result;
  const auto add_polygon = [&result, s1, s2, left_offset, right_offset](
                             const lanelet::ConstLanelets & lanes) {
    const double total_length = lanelet::geometry::length2d(lanelet::LaneletSequence(lanes));
    const auto expanded = autoware::experimental::lanelet2_utils::get_dirty_expanded_lanelets(
      lanes, left_offset, -right_offset);
    if (!expanded.has_value()) return;
    const auto polygon = autoware::experimental::lanelet2_utils::get_polygon_from_arc_length(
      expanded.value(), total_length - s2, total_length - s1);
    if (polygon.has_value()) result.push_back({polygon->basicPolygon(), lanes});
  };
  const auto previous_lanes = route_handler->getPreviousLanelets(target_lanes.front());
  if (previous_lanes.empty()) {
    add_polygon(target_lanes);
    return result;
  }
  for (const auto & previous_lane : previous_lanes) {
    const bool overlaps_current = std::any_of(
      current_lanes.begin(), current_lanes.end(), [&previous_lane](const auto & lane) {
        return lane.id() == previous_lane.id();
      });
    if (overlaps_current) {
      add_polygon(target_lanes);
      continue;
    }
    lanelet::ConstLanelets pushed = target_lanes;
    pushed.insert(pushed.begin(), previous_lane);
    const double total_length = lanelet::geometry::length2d(lanelet::LaneletSequence(pushed));
    if (total_length > s2) {
      add_polygon(pushed);
      continue;
    }
    auto recursive = make_rear_previous_detection_areas(
      current_lanes, pushed, s1, s2, route_handler, left_offset, right_offset);
    result.insert(result.end(), recursive.begin(), recursive.end());
  }
  return result;
}
}  // namespace autoware::planning_validator
