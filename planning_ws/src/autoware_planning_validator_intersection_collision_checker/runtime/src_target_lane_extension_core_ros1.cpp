#include "autoware/planning_validator_intersection_collision_checker/target_lane_extension_core_ros1.hpp"

#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>

namespace autoware::planning_validator::intersection_collision_checker
{
lanelet::ConstLanelets extend_target_lanelet(
  const autoware::route_handler::RouteHandler & route_handler,
  const lanelet::ConstLanelet & target_lanelet, const geometry_msgs::Pose & overlap_pose,
  const double detection_range)
{
  lanelet::ConstLanelets extended{target_lanelet};
  auto current_arc_length =
    autoware::experimental::lanelet2_utils::get_arc_coordinates(extended, overlap_pose).length;
  if (current_arc_length >= detection_range) return extended;

  lanelet::ConstLanelets predecessors{target_lanelet};
  while (current_arc_length < detection_range) {
    predecessors = route_handler.getPreviousLanelets(predecessors.front());
    if (predecessors.empty()) break;
    extended.push_back(predecessors.front());
    current_arc_length += lanelet::geometry::length2d(predecessors.front());
  }
  std::reverse(extended.begin(), extended.end());
  return extended;
}
}  // namespace autoware::planning_validator::intersection_collision_checker
