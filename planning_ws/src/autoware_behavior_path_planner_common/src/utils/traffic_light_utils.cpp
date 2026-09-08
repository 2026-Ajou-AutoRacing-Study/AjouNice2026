#include <autoware/behavior_path_planner_common/utils/traffic_light_utils.hpp>

#include <autoware/traffic_light_utils/traffic_light_utils.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/lanelet2_utils/nn_search.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <cmath>
#include <limits>

namespace autoware::behavior_path_planner::utils::traffic_light
{

double getDistanceToNextTrafficLight(
  const geometry_msgs::Pose & current_pose, const lanelet::ConstLanelets & lanelets)
{
  const auto current_lanelet = autoware::experimental::lanelet2_utils::get_closest_lanelet(
    lanelets, current_pose);
  if (!current_lanelet) return std::numeric_limits<double>::infinity();

  const auto ego_point = autoware::experimental::lanelet2_utils::from_ros(current_pose.position);
  const auto ego_arc = lanelet::geometry::toArcCoordinates(
    lanelet::utils::to2D(current_lanelet->centerline()), lanelet::utils::to2D(ego_point).basicPoint());
  double accumulated = 0.0;
  bool reached_current = false;
  for (const auto & lanelet : lanelets) {
    if (lanelet.id() == current_lanelet->id()) reached_current = true;
    if (!reached_current) continue;
    for (const auto & element : lanelet.regulatoryElementsAs<lanelet::TrafficLight>()) {
      const auto stop_line = element->stopLine();
      if (!stop_line || stop_line->empty()) continue;
      const auto stop_arc = lanelet::geometry::toArcCoordinates(
        lanelet::utils::to2D(lanelet.centerline()), lanelet::utils::to2D(stop_line->front()).basicPoint());
      const double distance = accumulated + stop_arc.length - ego_arc.length;
      if (distance > 0.0) return distance;
    }
    accumulated += lanelet::geometry::length3d(lanelet);
  }
  return std::numeric_limits<double>::infinity();
}

std::optional<double> calcDistanceToRedTrafficLight(
  const lanelet::ConstLanelets & lanelets,
  const autoware_internal_planning_msgs::PathWithLaneId & path,
  const std::shared_ptr<const PlannerData> & planner_data)
{
  if (!planner_data || !planner_data->self_odometry || path.points.empty()) return std::nullopt;
  for (const auto & lanelet : lanelets) {
    for (const auto & element : lanelet.regulatoryElementsAs<lanelet::TrafficLight>()) {
      const auto signal = planner_data->getTrafficSignal(element->id(), ros::Time::now());
      if (!signal || !autoware::traffic_light_utils::isTrafficSignalStop(lanelet, signal->signal)) continue;
      const auto stop_line = element->stopLine();
      if (!stop_line || stop_line->empty()) return std::nullopt;
      const auto & front = stop_line->front();
      const auto & back = stop_line->back();
      geometry_msgs::Point midpoint;
      midpoint.x = 0.5 * (front.x() + back.x());
      midpoint.y = 0.5 * (front.y() + back.y());
      midpoint.z = 0.5 * (front.z() + back.z());
      return autoware::motion_utils::calcSignedArcLength(
        path.points, planner_data->self_odometry->pose.pose.position, midpoint);
    }
  }
  return std::nullopt;
}

bool isStoppedAtRedTrafficLightWithinDistance(
  const lanelet::ConstLanelets & lanelets,
  const autoware_internal_planning_msgs::PathWithLaneId & path,
  const std::shared_ptr<const PlannerData> & planner_data, const double distance_threshold)
{
  if (!planner_data || !planner_data->self_odometry) return false;
  const auto & velocity = planner_data->self_odometry->twist.twist.linear;
  if (std::hypot(velocity.x, velocity.y) > 0.1) return false;
  const auto distance = calcDistanceToRedTrafficLight(lanelets, path, planner_data);
  return distance && *distance < distance_threshold;
}

bool isTrafficSignalStop(
  const lanelet::ConstLanelets & lanelets,
  const std::shared_ptr<const PlannerData> & planner_data)
{
  if (!planner_data) return false;

  const auto now = ros::Time::now();
  for (const auto & lanelet : lanelets) {
    for (const auto & element : lanelet.regulatoryElementsAs<lanelet::TrafficLight>()) {
      const auto signal = planner_data->getTrafficSignal(element->id(), now);
      if (!signal) continue;
      if (autoware::traffic_light_utils::isTrafficSignalStop(lanelet, signal->signal)) return true;
    }
  }
  return false;
}

}  // namespace autoware::behavior_path_planner::utils::traffic_light
