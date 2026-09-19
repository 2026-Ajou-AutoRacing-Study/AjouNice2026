#pragma once
#include <autoware_internal_planning_msgs/PlanningFactorArray.h>
#include <rviz/default_plugin/marker_array_display.h>
#include <rviz/properties/bool_property.h>
#include <ros/wall_timer.h>

namespace autoware::rviz_plugins {
class PlanningFactorRvizPlugin : public rviz::MarkerArrayDisplay {
  Q_OBJECT
public:
  PlanningFactorRvizPlugin();
  void onInitialize() override;
  void reset() override;
protected:
  void onEnable() override;
  void onDisable() override;
private:
  void onFactors(const autoware_internal_planning_msgs::PlanningFactorArray::ConstPtr & msg);
  void retryVehicleInfo(const ros::WallTimerEvent &);
  void subscribeFactors();
  void unsubscribeFactors();
  rviz::BoolProperty show_safety_factors_;
  ros::Subscriber factors_sub_;
  ros::Publisher marker_pub_;
  ros::WallTimer vehicle_info_timer_;
  double baselink2front_{0.0};
  bool vehicle_info_ready_{false};
  const std::string marker_topic_{"/planning_factor_rviz_plugin/markers"};
};
}  // namespace autoware::rviz_plugins
