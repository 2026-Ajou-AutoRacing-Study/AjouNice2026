#pragma once

#include <autoware/behavior_velocity_traffic_light_module/ros1_support.hpp>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace autoware::behavior_velocity_planner::signal_compliance
{
// Read-only video instrumentation, deliberately separate from planner logs.
// No subscriber means no serialization; no control code consumes this topic.
inline void publishVideoDecision(
  int64_t group, const ros::Time & stamp, bool stop, bool committed,
  double velocity, double target, double official, double braking, const char * reason)
{
  if (!ros::isInitialized() || !ros::isStarted()) return;
  static ros::NodeHandle node;
  static auto publisher = node.advertise<autoware_internal_debug_msgs::StringStamped>(
    "/perception/traffic_light_recognition/planner_decision_debug", 10);
  if (!publisher.getNumSubscribers()) return;
  std::ostringstream out;
  out << std::setprecision(17) << "{\"group_id\":" << group
      << ",\"require_stop\":" << (stop ? "true" : "false")
      << ",\"committed\":" << (committed ? "true" : "false");
  const auto number = [&](const char * name, double value) {
    out << ",\"" << name << "\":";
    if (std::isfinite(value)) out << value;
    else out << "null";
  };
  number("velocity_mps", velocity);
  number("target_m", target);
  number("official_m", official);
  number("brake_m", braking);
  out << ",\"reason\":\"" << reason << "\"}";
  autoware_internal_debug_msgs::StringStamped message;
  message.stamp.sec = stamp.sec;
  message.stamp.nanosec = stamp.nsec;
  message.data = out.str();
  publisher.publish(message);
}
}  // namespace autoware::behavior_velocity_planner::signal_compliance
