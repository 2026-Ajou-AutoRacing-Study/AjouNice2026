#ifndef AUTOWARE_PLANNING_VALIDATOR_REAR_LANELET_DETECTION_CORE_HPP_
#define AUTOWARE_PLANNING_VALIDATOR_REAR_LANELET_DETECTION_CORE_HPP_

#include <geometry_msgs/Pose.h>
#include <lanelet2_core/primitives/CompoundPolygon.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <memory>
#include <vector>

namespace autoware::route_handler
{
class RouteHandler;
}

namespace autoware::planning_validator
{
lanelet::ConstLanelet make_rear_half_lanelet(
  const lanelet::ConstLanelet & lanelet, bool is_right, double ignore_width_from_centerline,
  double expand_width_from_bound);

lanelet::BasicPolygon3d make_rear_detection_polygon(
  const lanelet::ConstLanelets & lanelets, const geometry_msgs::Pose & ego_pose,
  double forward_distance, double backward_distance);

struct RearDetectionArea
{
  lanelet::BasicPolygon3d polygon;
  lanelet::ConstLanelets lanes;
};

// Mirrors utils::cut_by_lanelets: subtract the combined current driving-lane
// polygon from every detection polygon and keep the first remaining component.
void cut_rear_detection_areas_by_current_lanes(
  const lanelet::ConstLanelets & current_lanes, std::vector<RearDetectionArea> & areas);

// Exact predecessor-walk shape of the adjacent-lane source. A null route
// handler is a readiness failure and therefore returns no detection areas.
std::vector<RearDetectionArea> make_rear_previous_detection_areas(
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & target_lanes,
  double s1, double s2, const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  double left_offset, double right_offset);

// R003 provider boundary: use the node-owned RouteHandler without creating an
// ownership alias merely to traverse predecessor lanelets.
std::vector<RearDetectionArea> make_rear_previous_detection_areas(
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & target_lanes,
  double s1, double s2, const autoware::route_handler::RouteHandler * route_handler,
  double left_offset, double right_offset);
}  // namespace autoware::planning_validator

#endif
