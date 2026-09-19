#include <autoware/behavior_velocity_planner_common/planner_data.hpp>

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <algorithm>
#include <cstdint>
#include <vector>

namespace autoware::behavior_velocity_planner
{
PlannerData::PlannerData(const ros::NodeHandle & node)
: vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo())
{
  node.param("max_accel", max_stop_acceleration_threshold, 0.0);
  node.param("max_jerk", max_stop_jerk_threshold, 0.0);
  node.param("system_delay", system_delay, 0.0);
  node.param("delay_response_time", delay_response_time, 0.0);
}

void PlannerData::updateOdometry(const nav_msgs::Odometry & message)
{
  auto odometry = std::make_shared<geometry_msgs::PoseStamped>();
  odometry->header = message.header;
  odometry->pose = message.pose.pose;
  current_odometry = odometry;

  auto velocity = std::make_shared<geometry_msgs::TwistStamped>();
  velocity->header = message.header;
  velocity->twist = message.twist.twist;
  current_velocity = velocity;
  velocity_buffer.push_front(*velocity);

  const auto now = ros::Time::now();
  while (!velocity_buffer.empty()) {
    const auto & stamp = velocity_buffer.back().header.stamp;
    const auto age = now >= stamp ? now - stamp : ros::Duration(0.0);
    if (age.toSec() <= velocity_buffer_time_sec) {
      break;
    }
    velocity_buffer.pop_back();
  }
}

void PlannerData::updateTrafficSignals(
  const autoware_perception_msgs::TrafficLightGroupArray & message)
{
  traffic_light_id_map_raw_.clear();
  const auto last_observed = traffic_light_id_map_last_observed_;
  traffic_light_id_map_last_observed_.clear();
  const ros::Time message_stamp{
    static_cast<std::uint32_t>(message.stamp.sec),
    static_cast<std::uint32_t>(message.stamp.nanosec)};
  for (const auto & signal : message.traffic_light_groups) {
    TrafficSignalStamped stamped{message_stamp, signal};
    const auto id = static_cast<lanelet::Id>(signal.traffic_light_group_id);
    traffic_light_id_map_raw_[id] = stamped;
    const bool unknown = std::any_of(
      signal.elements.begin(), signal.elements.end(), [](const auto & element) {
        return element.color == autoware_perception_msgs::TrafficLightElement::UNKNOWN;
      });
    const auto previous = last_observed.find(id);
    if (unknown && previous != last_observed.end()) {
      traffic_light_id_map_last_observed_[id] = previous->second;
      traffic_light_id_map_last_observed_[id].stamp = message_stamp;
    } else {
      traffic_light_id_map_last_observed_[id] = stamped;
    }
  }
}

bool PlannerData::isVehicleStopped(const double stop_duration) const
{
  if (velocity_buffer.empty()) {
    return false;
  }

  const auto now = ros::Time::now();
  std::vector<double> velocities;
  for (const auto & velocity : velocity_buffer) {
    velocities.push_back(velocity.twist.linear.x);
    const auto & stamp = velocity.header.stamp;
    const auto time_diff = now >= stamp ? now - stamp : ros::Duration(0.0);
    if (time_diff.toSec() >= stop_duration) {
      break;
    }
  }

  constexpr double stop_velocity = 1e-3;
  for (const auto velocity : velocities) {
    if (velocity >= stop_velocity) {
      return false;
    }
  }
  return true;
}

std::optional<TrafficSignalStamped> PlannerData::getTrafficSignal(
  const lanelet::Id id, const bool keep_last_observation) const
{
  const auto & signals =
    keep_last_observation ? traffic_light_id_map_last_observed_ : traffic_light_id_map_raw_;
  const auto found = signals.find(id);
  if (found == signals.end()) {
    return std::nullopt;
  }
  return found->second;
}
}  // namespace autoware::behavior_velocity_planner
