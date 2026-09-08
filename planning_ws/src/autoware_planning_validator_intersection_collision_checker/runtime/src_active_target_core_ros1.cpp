#include "autoware/planning_validator_intersection_collision_checker/active_target_core_ros1.hpp"

#include "autoware/planning_validator_intersection_collision_checker/target_lane_extension_core_ros1.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>

namespace autoware::planning_validator::intersection_collision_checker
{
std::optional<ActiveTarget> make_active_target(
  const autoware::route_handler::RouteHandler & route_handler,
  const lanelet::ConstLanelet & candidate, const autoware_planning_msgs::Trajectory & trajectory,
  const std::size_t ego_front_index, const double detection_range,
  const bool ego_fully_prioritized)
{
  if (ego_fully_prioritized) return std::nullopt;
  const auto overlap =
    trajectory_overlap_for_lanelet(candidate, trajectory, ego_front_index);
  if (!overlap.has_value()) return std::nullopt;
  const auto overlap_pose =
    autoware::experimental::lanelet2_utils::get_closest_center_pose(
      candidate,
      autoware::experimental::lanelet2_utils::from_ros(
        trajectory.points[overlap->first_index].pose));
  return ActiveTarget{
    candidate.id(),
    extend_target_lanelet(route_handler, candidate, overlap_pose, detection_range),
    overlap_pose,
    overlap.value()};
}
}  // namespace autoware::planning_validator::intersection_collision_checker
