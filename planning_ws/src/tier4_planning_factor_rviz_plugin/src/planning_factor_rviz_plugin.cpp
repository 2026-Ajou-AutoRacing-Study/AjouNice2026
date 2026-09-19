#include "planning_factor_rviz_plugin.hpp"
#include <autoware/motion_utils/marker/marker_helper.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace autoware::rviz_plugins {
PlanningFactorRvizPlugin::PlanningFactorRvizPlugin()
: show_safety_factors_("Show Safety Factors", true, "Display safety factor markers", this) {}

void PlanningFactorRvizPlugin::onInitialize() {
  rviz::MarkerArrayDisplay::onInitialize();
  setTopic(QString::fromStdString(marker_topic_), "visualization_msgs/MarkerArray");
  marker_pub_ = update_nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1, false);
  vehicle_info_timer_ = threaded_nh_.createWallTimer(ros::WallDuration(1.0), &PlanningFactorRvizPlugin::retryVehicleInfo, this);
}
void PlanningFactorRvizPlugin::onEnable() { rviz::MarkerArrayDisplay::onEnable(); subscribeFactors(); }
void PlanningFactorRvizPlugin::onDisable() { unsubscribeFactors(); rviz::MarkerArrayDisplay::onDisable(); }
void PlanningFactorRvizPlugin::reset() { rviz::MarkerArrayDisplay::reset(); }
void PlanningFactorRvizPlugin::subscribeFactors() {
  factors_sub_ = update_nh_.subscribe("planning_factors", 1, &PlanningFactorRvizPlugin::onFactors, this);
}
void PlanningFactorRvizPlugin::unsubscribeFactors() { factors_sub_.shutdown(); }
void PlanningFactorRvizPlugin::retryVehicleInfo(const ros::WallTimerEvent &) {
  double wheel_base = 0.0, front_overhang = 0.0;
  if (threaded_nh_.getParam("/adapi/node/vehicle_info/wheel_base", wheel_base) &&
      threaded_nh_.getParam("/adapi/node/vehicle_info/front_overhang", front_overhang)) {
    baselink2front_ = wheel_base + front_overhang; vehicle_info_ready_ = true; vehicle_info_timer_.stop();
  }
}
void PlanningFactorRvizPlugin::onFactors(const autoware_internal_planning_msgs::PlanningFactorArray::ConstPtr & msg) {
  visualization_msgs::MarkerArray output; int32_t id = 0;
  for (const auto & factor : msg->factors) {
    const std::string text = factor.module + (factor.detail.empty() ? "" : " (" + factor.detail + ")");
    for (const auto & point : factor.control_points) {
      visualization_msgs::MarkerArray walls;
      if (factor.behavior == autoware_internal_planning_msgs::PlanningFactor::STOP)
        walls = autoware::motion_utils::createStopVirtualWallMarker(point.pose, text, msg->header.stamp, id++, baselink2front_, "", factor.is_driving_forward);
      else if (factor.behavior == autoware_internal_planning_msgs::PlanningFactor::SLOW_DOWN || factor.behavior == autoware_internal_planning_msgs::PlanningFactor::UNKNOWN)
        walls = autoware::motion_utils::createSlowDownVirtualWallMarker(point.pose, text, msg->header.stamp, id++, baselink2front_, "", factor.is_driving_forward);
      output.markers.insert(output.markers.end(), walls.markers.begin(), walls.markers.end());
    }
    if (show_safety_factors_.getBool()) for (const auto & safety : factor.safety_factors.factors) for (const auto & point : safety.points) {
      visualization_msgs::Marker marker; marker.header = msg->header; marker.ns = factor.module + "_safety"; marker.id = id++; marker.type = visualization_msgs::Marker::SPHERE; marker.action = visualization_msgs::Marker::ADD; marker.pose.position = point; marker.pose.orientation.w = 1.0; marker.scale.x = marker.scale.y = marker.scale.z = 0.35; marker.color.a = 0.999; marker.color.g = safety.is_safe ? 0.9 : 0.24; marker.color.r = safety.is_safe ? 0.0 : 1.0; output.markers.push_back(marker);
    }
  }
  marker_pub_.publish(output);
}
}  // namespace autoware::rviz_plugins
PLUGINLIB_EXPORT_CLASS(autoware::rviz_plugins::PlanningFactorRvizPlugin, rviz::Display)
