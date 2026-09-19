#include "autoware/planning_validator/rear_shift_activation_core.hpp"

#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <boost/geometry.hpp>
#include <lanelet2_core/geometry/Lanelet.h>

namespace autoware::planning_validator
{
namespace
{
lanelet::BasicLineString2d axle_for_pose(const geometry_msgs::Pose & pose, const double vehicle_width)
{
  const auto left = autoware_utils_geometry::calc_offset_pose(pose, 0.0, 0.5 * vehicle_width, 0.0);
  const auto right = autoware_utils_geometry::calc_offset_pose(pose, 0.0, -0.5 * vehicle_width, 0.0);
  return {{left.position.x, left.position.y}, {right.position.x, right.position.y}};
}

lanelet::BasicLineString2d envelope_line(
  const geometry_msgs::Pose & left, const geometry_msgs::Pose & right)
{
  return {{left.position.x, left.position.y}, {right.position.x, right.position.y}};
}
}  // namespace

RearBehaviorActivationResult evaluate_rear_shift_activation(
  const RearShiftActivationInput & input, const RearShiftActivationConfig & config)
{
  const auto combined_lane = autoware::experimental::lanelet2_utils::combine_lanelets_shape(
    input.current_lanes);
  if (!combined_lane || input.trajectory.points.size() < 2U) return {};

  const auto & ego_pose = input.current_kinematics.pose.pose;
  const auto nearest_idx = autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    input.trajectory.points, ego_pose);
  const auto ego_axle = axle_for_pose(ego_pose, input.vehicle_info.vehicle_width_m);
  if (boost::geometry::intersects(
        ego_axle, lanelet::utils::to2D(combined_lane->leftBound()).basicLineString()) ||
      boost::geometry::intersects(
        ego_axle, lanelet::utils::to2D(combined_lane->rightBound()).basicLineString())) {
    return {};
  }

  const auto distance_to_stop = autoware::motion_utils::calcDistanceToForwardStopPoint(
    input.trajectory.points, ego_pose);
  const auto distance_residual = autoware::motion_utils::calcSignedArcLength(
    input.trajectory.points, ego_pose.position, nearest_idx);
  const auto reachable_line = envelope_line(
    input.motion_envelope.lookahead_left, input.motion_envelope.lookahead_right);
  const auto stoppable_line = envelope_line(
    input.motion_envelope.predicted_stop_left, input.motion_envelope.predicted_stop_right);

  for (size_t i = 0; i < input.trajectory.points.size(); ++i) {
    const auto & point = input.trajectory.points.at(i);
    const auto distance = autoware::motion_utils::calcSignedArcLength(
      input.trajectory.points, nearest_idx, i) + distance_residual;
    if (input.motion_envelope.lookahead_distance < distance && !input.is_unsafe_holding &&
        boost::geometry::within(reachable_line, combined_lane->polygon2d().basicPolygon())) {
      break;
    }
    if (input.motion_envelope.predicted_stop_distance > distance && !config.check_on_unstoppable &&
        !boost::geometry::within(stoppable_line, combined_lane->polygon2d().basicPolygon())) {
      break;
    }
    if (distance < input.vehicle_info.max_longitudinal_offset_m) continue;

    const auto axle = axle_for_pose(point.pose, input.vehicle_info.vehicle_width_m);
    if (boost::geometry::intersects(
          axle, lanelet::utils::to2D(combined_lane->leftBound()).basicLineString())) {
      const bool stopped_before_conflict = distance_to_stop &&
        *distance_to_stop + input.vehicle_info.max_longitudinal_offset_m < distance;
      return {stopped_before_conflict ? RearBehavior::NONE : RearBehavior::SHIFT_LEFT, distance};
    }
    if (boost::geometry::intersects(
          axle, lanelet::utils::to2D(combined_lane->rightBound()).basicLineString())) {
      const bool stopped_before_conflict = distance_to_stop &&
        *distance_to_stop + input.vehicle_info.max_longitudinal_offset_m < distance;
      return {stopped_before_conflict ? RearBehavior::NONE : RearBehavior::SHIFT_RIGHT, distance};
    }
  }
  return {};
}
}  // namespace autoware::planning_validator
