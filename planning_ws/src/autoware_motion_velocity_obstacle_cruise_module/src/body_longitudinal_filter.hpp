#ifndef OBSTACLE_CRUISE_BODY_LONGITUDINAL_FILTER_HPP_
#define OBSTACLE_CRUISE_BODY_LONGITUDINAL_FILTER_HPP_

#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <autoware_planning_msgs/TrajectoryPoint.h>
#include <autoware_perception_msgs/PredictedObject.h>

#include <optional>
#include <vector>

namespace autoware::motion_velocity_planner
{
struct BodyLongitudinalBounds
{
  double object_max_s;
  double ego_min_s;
  bool footprints_overlap;

  bool fully_behind() const
  {
    // Touching bodies and projection roundoff must not remove a target.
    return !footprints_overlap && object_max_s < ego_min_s - 1e-3;
  }
};

// Use the same world-frame shapes as the downstream polygon collision check.
// A negative reference-point distance alone does not mean a long vehicle has
// cleared Ego's rear bumper. Invalid geometry cannot establish that clearance.
std::optional<BodyLongitudinalBounds> body_longitudinal_bounds(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & trajectory,
  const geometry_msgs::Pose & ego_pose,
  const autoware_perception_msgs::PredictedObject & object,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle);
}  // namespace autoware::motion_velocity_planner
#endif
