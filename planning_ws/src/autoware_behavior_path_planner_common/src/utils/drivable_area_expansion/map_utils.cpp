#include <autoware/behavior_path_planner_common/utils/drivable_area_expansion/map_utils.hpp>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/primitives/LineString.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace autoware::behavior_path_planner::drivable_area_expansion
{
namespace
{
// The staged ROS1 Point2d registration lacks Boost's segment-point distance
// strategy.  This is the same Cartesian distance used by the upstream call.
double point_to_segment_distance(const Segment2d & segment, const Point2d & point)
{
  const auto & first = segment.first;
  const auto & second = segment.second;
  const double dx = second.x() - first.x();
  const double dy = second.y() - first.y();
  const double squared_length = dx * dx + dy * dy;
  const double projection = squared_length == 0.0 ? 0.0 : std::clamp(
    ((point.x() - first.x()) * dx + (point.y() - first.y()) * dy) / squared_length, 0.0, 1.0);
  return std::hypot(point.x() - (first.x() + projection * dx), point.y() - (first.y() + projection * dy));
}
}  // namespace

SegmentRtree extract_uncrossable_segments(const lanelet::LaneletMap & lanelet_map, const Point & ego_point,
  const DrivableAreaExpansionParameters & params)
{
  SegmentRtree uncrossable_segments_in_range;
  LineString2d line;
  const auto ego_p = Point2d{ego_point.x, ego_point.y};
  for (const auto & ls : lanelet_map.lineStringLayer) {
    if (has_types(ls, params.avoid_linestring_types)) {
      line.clear();
      for (const auto & p : ls) line.push_back(Point2d{p.x(), p.y()});
      for (auto segment_idx = 0LU; segment_idx + 1 < line.size(); ++segment_idx) {
        Segment2d segment = {line[segment_idx], line[segment_idx + 1]};
        if (point_to_segment_distance(segment, ego_p) < params.max_path_arc_length) {
          uncrossable_segments_in_range.insert(segment);
        }
      }
    }
  }
  return uncrossable_segments_in_range;
}

bool has_types(const lanelet::ConstLineString3d & ls,
  const std::vector<DrivableAreaExpansionParameters::LinestringType> & types)
{
  constexpr auto no_type = "";
  const auto type = ls.attributeOr(lanelet::AttributeName::Type, no_type);
  const auto subtype = ls.attributeOr(lanelet::AttributeName::Subtype, no_type);
  const auto matches_type = [&](const auto & t) { return t.matches(type, subtype); };
  return type != no_type && std::find_if(types.begin(), types.end(), matches_type) != types.end();
}
}  // namespace autoware::behavior_path_planner::drivable_area_expansion
