#pragma once

#include <autoware/behavior_velocity_planner_common/scene_module_manager_interface.hpp>
#include <autoware/objects_of_interest_marker_interface/objects_of_interest_marker_interface.hpp>
#include <autoware/rtc_interface/rtc_interface.hpp>

#include <limits>
#include <string>
#include <unordered_map>

namespace autoware::behavior_velocity_planner
{
using autoware::rtc_interface::RTCInterface;
using autoware::rtc_interface::UUID;
using tier4_rtc_msgs::State;

class SceneModuleInterfaceWithRTC : public SceneModuleInterface
{
public:
  SceneModuleInterfaceWithRTC(
    int64_t module_id, const ros::NodeHandle & node,
    std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
    std::shared_ptr<planning_factor_interface::PlanningFactorInterface> planning_factor_interface);

  void setActivation(bool activated) { activated_ = activated; }
  void setRTCEnabled(bool enabled) { rtc_enabled_ = enabled; }
  bool isActivated() const { return activated_; }
  bool isSafe() const { return safe_; }
  double getDistance() const { return distance_; }

protected:
  void setSafe(bool safe)
  {
    safe_ = safe;
    if (!rtc_enabled_) {
      setActivation(safe);
    }
  }
  void setDistance(double distance) { distance_ = distance; }

private:
  bool activated_{false};
  bool safe_{false};
  bool rtc_enabled_{true};
  double distance_{std::numeric_limits<double>::lowest()};
};

class SceneModuleManagerInterfaceWithRTC
: public SceneModuleManagerInterface<SceneModuleInterfaceWithRTC>
{
public:
  SceneModuleManagerInterfaceWithRTC(
    ros::NodeHandle & node, const char * module_name, bool enable_rtc = true);

  void plan(autoware_internal_planning_msgs::PathWithLaneId * path) override;

  static bool getEnableRTC(ros::NodeHandle & node, const std::string & param_name)
  {
    bool enable_all_auto_mode = false;
    bool enable_rtc = true;
    node.param("enable_all_modules_auto_mode", enable_all_auto_mode, false);
    node.param(param_name, enable_rtc, true);
    return enable_all_auto_mode ? false : enable_rtc;
  }

protected:
  RTCInterface rtc_interface_;
  void removeRTCStatus(const UUID & uuid) { rtc_interface_.removeCooperateStatus(uuid); }
  void updateRTCStatus(
    const UUID & uuid, bool safe, uint8_t state, double distance,
    const ros::Time & stamp, std::optional<bool> override_rtc_auto_mode = std::nullopt)
  {
    rtc_interface_.updateCooperateStatus(
      uuid, safe, state, distance, distance, stamp, false, override_rtc_auto_mode);
  }
  std::unordered_map<int64_t, UUID> map_uuid_;
  objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface objects_marker_;

  virtual void sendRTC(const ros::Time & stamp);
  virtual void setActivation();
  void deleteExpiredModules(const autoware_internal_planning_msgs::PathWithLaneId & path) override;
  UUID getUUID(int64_t module_id) const;
  void generateUUID(int64_t module_id);
  void removeUUID(int64_t module_id);
};
}  // namespace autoware::behavior_velocity_planner
