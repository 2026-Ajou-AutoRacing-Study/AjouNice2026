#include "autoware/hazard_lights_selector/node.hpp"
#include <builtin_interfaces/Time.h>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace autoware { namespace hazard_lights_selector {
namespace {
builtin_interfaces::Time toBuiltin(const ros::Time & time) {
  builtin_interfaces::Time result;
  result.sec = time.sec > static_cast<uint32_t>(std::numeric_limits<int32_t>::max()) ? std::numeric_limits<int32_t>::max() : static_cast<int32_t>(time.sec);
  result.nanosec = time.nsec;
  return result;
}
}
HazardLightsSelector::HazardLightsSelector(ros::NodeHandle nh, ros::NodeHandle private_nh) : nh_(nh), private_nh_(private_nh) {
  int update_rate = 10;
  if (!private_nh_.getParam("update_rate", update_rate) || update_rate <= 0) throw std::runtime_error("update_rate must be a positive integer");
  planning_ = nh_.subscribe("input/planning/hazard_lights_command", 1, &HazardLightsSelector::onPlanning, this);
  system_ = nh_.subscribe("input/system/hazard_lights_command", 1, &HazardLightsSelector::onSystem, this);
  output_ = nh_.advertise<autoware_vehicle_msgs::HazardLightsCommand>("output/hazard_lights_command", 1, false);
  timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate), &HazardLightsSelector::onTimer, this);
}
void HazardLightsSelector::onPlanning(const autoware_vehicle_msgs::HazardLightsCommand::ConstPtr & msg) { planning_command_ = msg; }
void HazardLightsSelector::onSystem(const autoware_vehicle_msgs::HazardLightsCommand::ConstPtr & msg) { system_command_ = msg; }
void HazardLightsSelector::onTimer(const ros::TimerEvent &) {
  autoware_vehicle_msgs::HazardLightsCommand result;
  result.stamp = toBuiltin(ros::Time::now());
  result.command = autoware_vehicle_msgs::HazardLightsCommand::DISABLE;
  if ((planning_command_ && planning_command_->command == autoware_vehicle_msgs::HazardLightsCommand::ENABLE) ||
      (system_command_ && system_command_->command == autoware_vehicle_msgs::HazardLightsCommand::ENABLE)) result.command = autoware_vehicle_msgs::HazardLightsCommand::ENABLE;
  output_.publish(result);
}
} }
