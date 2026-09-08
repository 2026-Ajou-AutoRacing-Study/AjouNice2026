#include <autoware/planning_validator_intersection_collision_checker/traffic_signal_lookup_core_ros1.hpp>

#include <algorithm>

namespace autoware::planning_validator::intersection_collision_checker_ros1
{
const autoware_perception_msgs::TrafficLightGroup * find_fresh_traffic_signal_group(
  const autoware_perception_msgs::TrafficLightGroupArray * signals, const int64_t group_id,
  const ros::Time & now, const double timeout_seconds)
{
  if (!signals) return nullptr;
  const ros::Time stamp(signals->stamp.sec, signals->stamp.nanosec);
  if ((now - stamp).toSec() > timeout_seconds) return nullptr;
  const auto it = std::find_if(
    signals->traffic_light_groups.begin(), signals->traffic_light_groups.end(),
    [group_id](const auto & group) { return group.traffic_light_group_id == group_id; });
  return it == signals->traffic_light_groups.end() ? nullptr : &*it;
}
}  // namespace autoware::planning_validator::intersection_collision_checker_ros1
