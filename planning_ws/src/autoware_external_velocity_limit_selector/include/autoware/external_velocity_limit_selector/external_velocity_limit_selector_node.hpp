#ifndef AUTOWARE_EXTERNAL_VELOCITY_LIMIT_SELECTOR_NODE_HPP_
#define AUTOWARE_EXTERNAL_VELOCITY_LIMIT_SELECTOR_NODE_HPP_

#include <autoware_internal_debug_msgs/StringStamped.h>
#include <autoware_internal_planning_msgs/VelocityLimit.h>
#include <autoware_internal_planning_msgs/VelocityLimitClearCommand.h>
#include <ros/ros.h>

#include <string>
#include <unordered_map>

namespace autoware { namespace external_velocity_limit_selector {
using VelocityLimit = autoware_internal_planning_msgs::VelocityLimit;
using VelocityLimitClearCommand = autoware_internal_planning_msgs::VelocityLimitClearCommand;
using VelocityLimitConstraints = autoware_internal_planning_msgs::VelocityLimitConstraints;
using VelocityLimitTable = std::unordered_map<std::string, VelocityLimit>;
struct Constraints { double min_acc; double max_acc; double min_jerk; double max_jerk; };
struct Params { double max_vel; Constraints normal; Constraints limit; };

class ExternalVelocityLimitSelectorNode {
public:
  ExternalVelocityLimitSelectorNode(ros::NodeHandle nh, ros::NodeHandle private_nh);
private:
  void onVelocityLimitFromAPI(const VelocityLimit::ConstPtr & msg);
  void onVelocityLimitFromInternal(const VelocityLimit::ConstPtr & msg);
  void onVelocityLimitClearCommand(const VelocityLimitClearCommand::ConstPtr & msg);
  void publishVelocityLimit(const VelocityLimit & velocity_limit);
  void publishDebugString();
  void setVelocityLimitFromAPI(const VelocityLimit & velocity_limit);
  void setVelocityLimitFromInternal(const VelocityLimit & velocity_limit);
  void clearVelocityLimit(const std::string & sender);
  void updateVelocityLimit();
  bool refreshParams();
  VelocityLimit getCurrentVelocityLimit() const { return hardest_limit_; }
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber sub_external_velocity_limit_from_api_;
  ros::Subscriber sub_external_velocity_limit_from_internal_;
  ros::Subscriber sub_velocity_limit_clear_command_;
  ros::Publisher pub_external_velocity_limit_;
  ros::Publisher pub_debug_string_;
  Params params_{};
  VelocityLimit hardest_limit_{};
  VelocityLimitTable velocity_limit_table_;
};
} }  // namespace autoware::external_velocity_limit_selector
#endif
