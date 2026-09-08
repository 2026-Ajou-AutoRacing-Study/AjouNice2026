#pragma once

#include <autoware/behavior_path_planner_common/interface/scene_module_manager_core.hpp>
#include <autoware/behavior_path_planner_common/interface/scene_module_interface.hpp>
#include <autoware/objects_of_interest_marker_interface/objects_of_interest_marker_interface.hpp>
#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <autoware/rtc_interface/rtc_interface.hpp>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::behavior_path_planner
{

// ROS1 provider-bound part of the upstream manager.  It owns the shared
// interfaces; module-specific RTC UUID updates and marker insertion remain in
// the SceneModuleCore provider-integration slice.
class SceneModuleManagerRos1 : public SceneModuleManagerCore
{
public:
  struct MarkerArrays {
    visualization_msgs::MarkerArray info;
    visualization_msgs::MarkerArray debug;
    visualization_msgs::MarkerArray drivable_lanes;
  };

  SceneModuleManagerRos1(ros::NodeHandle * node, std::string name)
  : SceneModuleManagerCore(std::move(name)), node_(node)
  {
  }

  bool initInterface(const std::vector<std::string> & rtc_types, const bool enable_rtc = false)
  {
    if (!node_) return false;
    // Match the source manager contract: RTC is opt-in per module, while the
    // global auto-mode switch disables manual RTC approval for every module.
    bool resolved_enable_rtc = enable_rtc;
    bool enable_all_modules_auto_mode = false;
    const bool has_global_auto_mode =
      node_->getParam("enable_all_modules_auto_mode", enable_all_modules_auto_mode);
    bool module_enable_rtc = false;
    const bool has_module_enable_rtc = node_->getParam(name() + "/enable_rtc", module_enable_rtc);
    if (has_global_auto_mode && enable_all_modules_auto_mode) {
      resolved_enable_rtc = false;
    } else if (has_module_enable_rtc) {
      resolved_enable_rtc = module_enable_rtc;
    }
    planning_factor_interface_ = std::make_shared<autoware::planning_factor_interface::PlanningFactorInterface>(node_, name());
    for (const auto & type : rtc_types) {
      const auto interface_name = type.empty() ? name() : name() + "_" + type;
      rtc_interfaces_.emplace(
        type, std::make_shared<autoware::rtc_interface::RTCInterface>(node_, interface_name, resolved_enable_rtc));
      marker_interfaces_.emplace(
        type, std::make_shared<autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface>(node_, interface_name));
    }
    processing_time_publisher_ = node_->advertise<autoware_utils::ProcessingTimeDetail>(
      "debug/processing_time_detail_ms/" + name(), 20);
    info_marker_publisher_ = node_->advertise<visualization_msgs::MarkerArray>("info/" + name(), 20);
    debug_marker_publisher_ = node_->advertise<visualization_msgs::MarkerArray>("debug/" + name(), 20);
    drivable_lanes_publisher_ = node_->advertise<visualization_msgs::MarkerArray>("drivable_lanes/" + name(), 20);
    return true;
  }

  void registerNewModule(
    const std::shared_ptr<SceneModuleCore> & module, const BehaviorModuleOutput & previous_output)
  {
    SceneModuleManagerCore::registerNewModule(module, previous_output);
    const auto provider_module = std::dynamic_pointer_cast<SceneModuleInterface>(module);
    if (provider_module && processing_time_publisher_) {
      provider_module->getTimeKeeper()->add_reporter(processing_time_publisher_);
    }
  }

  void publishRTCStatus()
  {
    for (const auto & [type, interface] : rtc_interfaces_) {
      if (!interface) continue;
      interface->removeExpiredCooperateStatus();
      interface->publishCooperateStatus(ros::Time::now());
    }
  }

  void publish_planning_factors()
  {
    if (planning_factor_interface_) planning_factor_interface_->publish();
  }

  void publishObjectsOfInterest()
  {
    for (const auto & [type, interface] : marker_interfaces_) {
      if (interface) interface->publishMarkerArray();
    }
  }

  MarkerArrays aggregateMarkers() const
  {
    MarkerArrays aggregated;
    const auto append = [](const visualization_msgs::MarkerArray & source,
                           visualization_msgs::MarkerArray * destination, const uint32_t offset) {
      for (auto marker : source.markers) {
        marker.id += offset;
        destination->markers.push_back(marker);
      }
    };
    const auto marker_offset = static_cast<uint32_t>(std::numeric_limits<uint8_t>::max());
    uint32_t marker_id = marker_offset;
    const auto modules = activeModules();
    for (const auto & module : modules) {
      const auto provider = std::dynamic_pointer_cast<SceneModuleInterface>(module);
      if (!provider) continue;
      append(provider->getInfoMarkers(), &aggregated.info, marker_id);
      append(provider->getDebugMarkers(), &aggregated.debug, marker_id);
      append(provider->getDrivableLanesMarkers(), &aggregated.drivable_lanes, marker_id);
      marker_id += marker_offset;
    }
    if (modules.empty()) {
      if (const auto provider = dynamic_cast<SceneModuleInterface *>(idleModule())) {
        append(provider->getInfoMarkers(), &aggregated.info, 0);
        append(provider->getDebugMarkers(), &aggregated.debug, 0);
        append(provider->getDrivableLanesMarkers(), &aggregated.drivable_lanes, 0);
      }
    }
    return aggregated;
  }

  void publishMarkers() const
  {
    const auto markers = aggregateMarkers();
    if (info_marker_publisher_) info_marker_publisher_.publish(markers.info);
    if (debug_marker_publisher_) debug_marker_publisher_.publish(markers.debug);
    if (drivable_lanes_publisher_) drivable_lanes_publisher_.publish(markers.drivable_lanes);
  }

  bool initProviders(const std::vector<std::string> & rtc_types, const bool enable_rtc = false)
  {
    return initInterface(rtc_types, enable_rtc);
  }
  void publishRtcStatus() { publishRTCStatus(); }
  void publishPlanningFactors() { publish_planning_factors(); }

  size_t rtcInterfaceCount() const { return rtc_interfaces_.size(); }
  size_t markerInterfaceCount() const { return marker_interfaces_.size(); }
  bool hasPlanningFactorInterface() const { return static_cast<bool>(planning_factor_interface_); }

protected:
  ros::NodeHandle * node_{nullptr};
  std::shared_ptr<autoware::planning_factor_interface::PlanningFactorInterface> planning_factor_interface_;
  std::unordered_map<std::string, std::shared_ptr<autoware::rtc_interface::RTCInterface>> rtc_interfaces_;
  std::unordered_map<std::string, std::shared_ptr<autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface>> marker_interfaces_;
  ros::Publisher processing_time_publisher_;
  ros::Publisher info_marker_publisher_;
  ros::Publisher debug_marker_publisher_;
  ros::Publisher drivable_lanes_publisher_;
};

}  // namespace autoware::behavior_path_planner
