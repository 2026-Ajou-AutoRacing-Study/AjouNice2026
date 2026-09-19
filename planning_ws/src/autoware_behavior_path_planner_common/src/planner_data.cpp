#include <autoware/behavior_path_planner_common/planner_data.hpp>

#include <autoware/route_handler/route_handler.hpp>

namespace autoware::behavior_path_planner
{
void PlannerData::ensureRouteHandler()
{
  if (!route_handler) route_handler = std::make_shared<autoware::route_handler::RouteHandler>();
}

void PlannerData::updateTrafficSignals(
  const autoware_perception_msgs::TrafficLightGroupArray & message)
{
  traffic_light_id_map.clear();
  for (const auto & signal : message.traffic_light_groups) {
    const ros::Time stamp{
      static_cast<uint32_t>(message.stamp.sec), static_cast<uint32_t>(message.stamp.nanosec)};
    traffic_light_id_map.emplace(signal.traffic_light_group_id, TrafficSignalStamped{stamp, signal});
  }
}

std::optional<TrafficSignalStamped> PlannerData::getTrafficSignal(
  const int64_t id, const ros::Time & now) const
{
  const auto found = traffic_light_id_map.find(id);
  if (found == traffic_light_id_map.end()) return std::nullopt;
  if (now >= found->second.stamp &&
      (now - found->second.stamp).toSec() > parameters.traffic_light_signal_timeout) {
    return std::nullopt;
  }
  return found->second;
}
}  // namespace autoware::behavior_path_planner
