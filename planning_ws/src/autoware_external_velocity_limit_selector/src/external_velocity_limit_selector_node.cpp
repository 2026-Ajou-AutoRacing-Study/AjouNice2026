#include "autoware/external_velocity_limit_selector/external_velocity_limit_selector_node.hpp"
#include <builtin_interfaces/Time.h>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace autoware { namespace external_velocity_limit_selector {
namespace {
builtin_interfaces::Time rosTimeToBuiltinTime(const ros::Time & time) {
  builtin_interfaces::Time result;
  result.sec = time.sec > static_cast<uint32_t>(std::numeric_limits<int32_t>::max()) ?
    std::numeric_limits<int32_t>::max() : static_cast<int32_t>(time.sec);
  result.nanosec = time.nsec;
  return result;
}
VelocityLimit getHardestLimit(const VelocityLimitTable & values, const Params & params) {
  VelocityLimit hardest{};
  hardest.max_velocity = params.max_vel;
  VelocityLimitConstraints normal{};
  normal.max_acceleration = params.normal.max_acc;
  normal.min_acceleration = params.normal.min_acc;
  normal.min_jerk = params.normal.min_jerk;
  normal.max_jerk = params.normal.max_jerk;
  double max_velocity = params.max_vel, max_jerk = 0.0, max_acceleration = 0.0;
  std::string max_acceleration_key;
  size_t constraint_count = 0, acceleration_constraint_count = 0;
  for (const auto & item : values) {
    const double velocity = std::isfinite(item.second.max_velocity) ? item.second.max_velocity : params.max_vel;
    if (velocity < max_velocity) {
      hardest.stamp = item.second.stamp; hardest.sender = item.first;
      hardest.max_velocity = velocity; max_velocity = velocity;
    }
    const VelocityLimitConstraints constraints = item.second.use_constraints && std::isfinite(item.second.constraints.max_jerk) ? item.second.constraints : normal;
    if (item.second.use_constraints) {
      ++constraint_count;
      if (item.second.constraints.max_acceleration > normal.max_acceleration) {
        ++acceleration_constraint_count; max_acceleration_key = item.first;
      }
    }
    if (max_acceleration < item.second.constraints.max_acceleration) {
      max_acceleration = item.second.constraints.max_acceleration; max_acceleration_key = item.first;
    }
    if (max_jerk < constraints.max_jerk) {
      hardest.constraints = constraints; hardest.use_constraints = true; max_jerk = constraints.max_jerk;
    }
  }
  if (constraint_count && constraint_count == acceleration_constraint_count && values.count(max_acceleration_key)) {
    hardest.constraints = values.at(max_acceleration_key).constraints; hardest.use_constraints = true;
  }
  return hardest;
}
std::string getDebugString(const VelocityLimitTable & values) {
  std::ostringstream out; out << std::boolalpha << std::fixed << std::setprecision(2);
  for (const auto & item : values) {
    out << "[" << item.first << "](" << static_cast<bool>(item.second.use_constraints) << ","
        << item.second.max_velocity << "," << item.second.constraints.min_acceleration << ","
        << item.second.constraints.min_jerk << "," << item.second.constraints.max_jerk << ")";
  }
  return out.str();
}
}  // namespace

ExternalVelocityLimitSelectorNode::ExternalVelocityLimitSelectorNode(ros::NodeHandle nh, ros::NodeHandle private_nh)
: nh_(nh), private_nh_(private_nh) {
  // ROS1 subscriber queues retain source depths; a latched peer is required for late join delivery.
  sub_external_velocity_limit_from_api_ = nh_.subscribe("input/velocity_limit_from_api", 1, &ExternalVelocityLimitSelectorNode::onVelocityLimitFromAPI, this);
  sub_external_velocity_limit_from_internal_ = nh_.subscribe("input/velocity_limit_from_internal", 10, &ExternalVelocityLimitSelectorNode::onVelocityLimitFromInternal, this);
  sub_velocity_limit_clear_command_ = nh_.subscribe("input/velocity_limit_clear_command_from_internal", 10, &ExternalVelocityLimitSelectorNode::onVelocityLimitClearCommand, this);
  pub_external_velocity_limit_ = nh_.advertise<VelocityLimit>("output/external_velocity_limit", 1, false);
  pub_debug_string_ = nh_.advertise<autoware_internal_debug_msgs::StringStamped>("output/debug", 1, false);
  if (!refreshParams()) throw std::runtime_error("external_velocity_limit_selector requires nine typed ROS parameters");
  updateVelocityLimit();
}
bool ExternalVelocityLimitSelectorNode::refreshParams() {
  Params next{};
  const bool complete = private_nh_.getParam("max_vel", next.max_vel) &&
    private_nh_.getParam("normal/min_acc", next.normal.min_acc) && private_nh_.getParam("normal/max_acc", next.normal.max_acc) &&
    private_nh_.getParam("normal/min_jerk", next.normal.min_jerk) && private_nh_.getParam("normal/max_jerk", next.normal.max_jerk) &&
    private_nh_.getParam("limit/min_acc", next.limit.min_acc) && private_nh_.getParam("limit/max_acc", next.limit.max_acc) &&
    private_nh_.getParam("limit/min_jerk", next.limit.min_jerk) && private_nh_.getParam("limit/max_jerk", next.limit.max_jerk);
  if (!complete) { ROS_ERROR("missing or non-double selector parameter under %s", private_nh_.getNamespace().c_str()); return false; }
  params_ = next; return true;
}
void ExternalVelocityLimitSelectorNode::onVelocityLimitFromAPI(const VelocityLimit::ConstPtr & msg) { setVelocityLimitFromAPI(*msg); publishVelocityLimit(getCurrentVelocityLimit()); publishDebugString(); }
void ExternalVelocityLimitSelectorNode::onVelocityLimitFromInternal(const VelocityLimit::ConstPtr & msg) { setVelocityLimitFromInternal(*msg); publishVelocityLimit(getCurrentVelocityLimit()); publishDebugString(); }
void ExternalVelocityLimitSelectorNode::onVelocityLimitClearCommand(const VelocityLimitClearCommand::ConstPtr & msg) { if (!msg->command) return; clearVelocityLimit(msg->sender); publishVelocityLimit(getCurrentVelocityLimit()); publishDebugString(); }
void ExternalVelocityLimitSelectorNode::publishVelocityLimit(const VelocityLimit & value) { pub_external_velocity_limit_.publish(value); }
void ExternalVelocityLimitSelectorNode::publishDebugString() { autoware_internal_debug_msgs::StringStamped debug{}; debug.stamp = rosTimeToBuiltinTime(ros::Time::now()); debug.data = getDebugString(velocity_limit_table_); pub_debug_string_.publish(debug); }
void ExternalVelocityLimitSelectorNode::setVelocityLimitFromAPI(const VelocityLimit & value) { velocity_limit_table_["api"] = value; updateVelocityLimit(); }
void ExternalVelocityLimitSelectorNode::setVelocityLimitFromInternal(const VelocityLimit & value) { velocity_limit_table_[value.sender] = value; updateVelocityLimit(); }
void ExternalVelocityLimitSelectorNode::clearVelocityLimit(const std::string & sender) { velocity_limit_table_.erase(sender); updateVelocityLimit(); }
void ExternalVelocityLimitSelectorNode::updateVelocityLimit() {
  // ROS1 has no SetParameters equivalent. Input/clear callbacks are the documented reload points.
  if (!refreshParams()) return;
  if (velocity_limit_table_.empty()) { VelocityLimit initial{}; initial.stamp = rosTimeToBuiltinTime(ros::Time::now()); initial.max_velocity = params_.max_vel; hardest_limit_ = initial; return; }
  hardest_limit_ = getHardestLimit(velocity_limit_table_, params_);
}
} }  // namespace autoware::external_velocity_limit_selector
