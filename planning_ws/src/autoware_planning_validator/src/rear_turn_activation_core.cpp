#include "autoware/planning_validator/rear_turn_activation_core.hpp"

#include <angles/angles.h>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <boost/geometry.hpp>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
#include <string>

namespace autoware::planning_validator
{
namespace
{
std::optional<lanelet::ConstLanelet> sibling_straight_lanelet(
  const lanelet::ConstLanelet & assigned_lane,
  const lanelet::routing::RoutingGraphConstPtr & routing_graph)
{
  if (!routing_graph) return std::nullopt;
  for (const auto & previous : routing_graph->previous(assigned_lane)) {
    for (const auto & following : routing_graph->following(previous)) {
      const std::string turn_direction = following.attributeOr("turn_direction", "else");
      if (turn_direction == "straight") return following;
    }
  }
  return std::nullopt;
}

lanelet::BasicLineString2d axle_for_pose(const geometry_msgs::Pose & pose, const double vehicle_width)
{
  const auto left = autoware_utils_geometry::calc_offset_pose(pose, 0.0, 0.5 * vehicle_width, 0.0);
  const auto right = autoware_utils_geometry::calc_offset_pose(pose, 0.0, -0.5 * vehicle_width, 0.0);
  return {{left.position.x, left.position.y}, {right.position.x, right.position.y}};
}

lanelet::BasicLineString2d envelope_line(const geometry_msgs::Pose & left, const geometry_msgs::Pose & right)
{
  return {{left.position.x, left.position.y}, {right.position.x, right.position.y}};
}

std::optional<geometry_msgs::Point> conflict_point(
  const autoware_planning_msgs::Trajectory & trajectory,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const lanelet::ConstLanelet & sibling, const bool is_right)
{
  const auto bound = is_right ? sibling.rightBound() : sibling.leftBound();
  for (const auto & point : trajectory.points) {
    if (boost::geometry::intersects(
          axle_for_pose(point.pose, vehicle_info.vehicle_width_m),
          lanelet::utils::to2D(bound).basicLineString())) {
      return point.pose.position;
    }
  }
  return std::nullopt;
}
}  // namespace

RearBehaviorActivationResult evaluate_rear_turn_activation(
  const RearTurnActivationInput & input, const RearTurnActivationConfig & config)
{
  if (!input.route_handler || input.current_lanes.empty() || input.trajectory.points.size() < 2U) return {};
  const auto routing_graph = input.route_handler->getRoutingGraphPtr();
  if (!routing_graph) return {};
  const auto & ego_pose = input.current_kinematics.pose.pose;
  const auto ego_arc = autoware::experimental::lanelet2_utils::get_arc_coordinates(
    input.current_lanes, ego_pose);
  const auto distance_to_stop = autoware::motion_utils::calcDistanceToForwardStopPoint(
    input.trajectory.points, ego_pose);

  double total_length = 0.0;
  for (const auto & lane : input.current_lanes) {
    const auto distance = total_length - ego_arc.length - input.vehicle_info.max_longitudinal_offset_m;
    total_length += lanelet::geometry::length2d(lane);
    if (total_length - input.vehicle_info.min_longitudinal_offset_m < ego_arc.length) continue;
    const std::string direction = lane.attributeOr("turn_direction", "none");
    const bool left = direction == "left" && config.check_left;
    const bool right = direction == "right" && config.check_right;
    if (!left && !right) continue;

    if (lane.centerline().size() < 2U) continue;
    const auto & last = lane.centerline()[lane.centerline().size() - 1U];
    const auto & before_last = lane.centerline()[lane.centerline().size() - 2U];
    const auto lane_yaw = std::atan2(last.y() - before_last.y(), last.x() - before_last.x());
    const auto yaw_difference = angles::shortest_angular_distance(lane_yaw, tf2::getYaw(ego_pose.orientation));
    if ((left && yaw_difference > -config.yaw_threshold) ||
        (right && yaw_difference < config.yaw_threshold)) {
      continue;
    }

    const auto sibling = sibling_straight_lanelet(lane, routing_graph);
    const auto exceeds_deadline = [&]() {
      if (!sibling) return distance < input.motion_envelope.predicted_stop_distance;
      const auto conflict = conflict_point(input.trajectory, input.vehicle_info, *sibling, right);
      if (!conflict) return distance < input.motion_envelope.predicted_stop_distance;
      return autoware::motion_utils::calcSignedArcLength(
               input.trajectory.points, ego_pose.position, *conflict) -
               input.vehicle_info.max_longitudinal_offset_m < input.motion_envelope.predicted_stop_distance;
    };
    if (!config.check_on_unstoppable && exceeds_deadline()) continue;

    const bool reachable = distance < input.motion_envelope.lookahead_distance || input.is_unsafe_holding;
    if (distance_to_stop && *distance_to_stop < distance + 0.5) return {};
    return {reachable ? (left ? RearBehavior::TURN_LEFT : RearBehavior::TURN_RIGHT) : RearBehavior::NONE,
      distance};
  }
  return {};
}
}  // namespace autoware::planning_validator
