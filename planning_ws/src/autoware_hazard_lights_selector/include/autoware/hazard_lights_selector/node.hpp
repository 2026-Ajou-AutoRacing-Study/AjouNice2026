#ifndef AUTOWARE_HAZARD_LIGHTS_SELECTOR_NODE_HPP_
#define AUTOWARE_HAZARD_LIGHTS_SELECTOR_NODE_HPP_

#include <autoware_vehicle_msgs/HazardLightsCommand.h>
#include <ros/ros.h>

namespace autoware { namespace hazard_lights_selector {
class HazardLightsSelector {
public:
  HazardLightsSelector(ros::NodeHandle nh, ros::NodeHandle private_nh);
private:
  void onPlanning(const autoware_vehicle_msgs::HazardLightsCommand::ConstPtr & msg);
  void onSystem(const autoware_vehicle_msgs::HazardLightsCommand::ConstPtr & msg);
  void onTimer(const ros::TimerEvent &);
  ros::NodeHandle nh_, private_nh_;
  ros::Subscriber planning_, system_;
  ros::Publisher output_;
  ros::Timer timer_;
  autoware_vehicle_msgs::HazardLightsCommand::ConstPtr planning_command_, system_command_;
};
} }
#endif
