#pragma once

#include <autoware/behavior_path_planner_common/interface/scene_module_core.hpp>
#include <autoware/behavior_path_planner_common/marker_utils/utils.hpp>
#include <autoware/objects_of_interest_marker_interface/objects_of_interest_marker_interface.hpp>
#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <autoware/rtc_interface/rtc_interface.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/ros/uuid_helper.hpp>
#include <autoware_utils/system/time_keeper.hpp>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <memory>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>

namespace autoware::behavior_path_planner
{
// ROS2 SceneModuleInterface exposes a clock object to concrete modules for
// both RTC timestamps and throttled diagnostics.  Keep that ownership in the
// ROS1 provider layer instead of letting each module synthesize its own clock.
class Ros1SceneModuleClock
{
public:
  ros::Time now() const { return ros::Time::now(); }
};

// ROS1 provider-bearing companion to SceneModuleCore.  It deliberately keeps
// the provider objects owned by SceneModuleManagerRos1 alive in every module
// instance instead of silently dropping the source constructor arguments.
class SceneModuleInterface : public SceneModuleCore
{
public:
  using RTCInterfaceMap = std::unordered_map<std::string, std::shared_ptr<autoware::rtc_interface::RTCInterface>>;
  using MarkerInterfaceMap = std::unordered_map<
    std::string,
    std::shared_ptr<autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface>>;

  SceneModuleInterface(
    std::string name, RTCInterfaceMap rtc_interfaces = {}, MarkerInterfaceMap marker_interfaces = {},
    std::shared_ptr<autoware::planning_factor_interface::PlanningFactorInterface> planning_factor_interface = nullptr,
    ModuleStatus initial = ModuleStatus::IDLE);

  size_t rtcInterfaceCount() const { return rtc_interfaces_.size(); }
  size_t markerInterfaceCount() const { return marker_interfaces_.size(); }
  bool hasPlanningFactorInterface() const { return static_cast<bool>(planning_factor_interface_); }
  std::shared_ptr<autoware_utils::TimeKeeper> getTimeKeeper() const { return time_keeper_; }
  visualization_msgs::MarkerArray getInfoMarkers() const { return info_marker_; }
  visualization_msgs::MarkerArray getDebugMarkers() const { return debug_marker_; }
  visualization_msgs::MarkerArray getDrivableLanesMarkers() const { return drivable_lanes_marker_; }

  void lockRTCCommand()
  {
    for (const auto & [module_name, interface] : rtc_interfaces_) {
      if (interface) interface->lockCommandUpdate();
    }
  }

  void unlockRTCCommand()
  {
    for (const auto & [module_name, interface] : rtc_interfaces_) {
      if (interface) interface->unlockCommandUpdate();
    }
  }

  void updateRTCStatus(const double start_distance, const double finish_distance)
  {
    for (const auto & [module_name, interface] : rtc_interfaces_) {
      if (!interface) continue;
      const auto & uuid = rtc_uuids_.at(module_name);
      const auto state = !interface->isRegistered(uuid) || isWaitingApproval()
                           ? autoware::rtc_interface::State::WAITING_FOR_EXECUTION
                           : autoware::rtc_interface::State::RUNNING;
      interface->updateCooperateStatus(
        uuid, isExecutionReady(), state, start_distance, finish_distance, ros::Time::now());
    }
  }

  bool hasRegisteredRTCStatus() const
  {
    for (const auto & [module_name, interface] : rtc_interfaces_) {
      if (interface && interface->isRegistered(rtc_uuids_.at(module_name))) return true;
    }
    return false;
  }

  // Read-only diagnostics snapshot. Nonvirtual and inline: no layout/vtable change.
  std::unordered_map<std::string, unique_identifier_msgs::UUID> getRtcUuids() const
  {
    return rtc_uuids_;
  }

  void update_rtc_status(
    double start_distance, double finish_distance,
    std::optional<bool> safe = std::nullopt, std::optional<uint8_t> state = std::nullopt)
  {
    for (const auto & [module_name, interface] : rtc_interfaces_) {
      if (!interface) continue;
      const auto & uuid = rtc_uuids_.at(module_name);
      const auto default_state = !interface->isRegistered(uuid) || isWaitingApproval()
        ? autoware::rtc_interface::State::WAITING_FOR_EXECUTION
        : autoware::rtc_interface::State::RUNNING;
      interface->updateCooperateStatus(uuid, safe.value_or(isExecutionReady()),
        state.value_or(default_state), start_distance, finish_distance, ros::Time::now());
    }
  }

  template <class PathPoints>
  void update_rtc_status(
    const PathPoints & points, const geometry_msgs::Point & reference,
    const geometry_msgs::Point & start, const geometry_msgs::Point & end,
    std::optional<bool> safe = std::nullopt, std::optional<uint8_t> state = std::nullopt)
  {
    if (points.empty()) {
      update_rtc_status(0.0, 0.0, safe, state);
      return;
    }
    update_rtc_status(
      autoware::motion_utils::calcSignedArcLength(points, reference, start),
      autoware::motion_utils::calcSignedArcLength(points, reference, end), safe, state);
  }

  bool is_rtc_force_activated() const
  {
    return std::any_of(rtc_interfaces_.begin(), rtc_interfaces_.end(), [&](const auto & rtc) {
      return rtc.second && rtc.second->isForceActivated(rtc_uuids_.at(rtc.first));
    });
  }

  bool is_rtc_force_deactivated() const
  {
    return std::any_of(rtc_interfaces_.begin(), rtc_interfaces_.end(), [&](const auto & rtc) {
      return rtc.second && rtc.second->isForceDeactivated(rtc_uuids_.at(rtc.first));
    });
  }

  void removeRTCStatus()
  {
    for (const auto & [module_name, interface] : rtc_interfaces_) {
      if (interface) interface->clearCooperateStatus();
    }
  }

  void updateRTCStatusForSuccess()
  {
    for (const auto & [module_name, interface] : rtc_interfaces_) {
      if (!interface) continue;
      const auto & uuid = rtc_uuids_.at(module_name);
      if (interface->isRegistered(uuid)) {
        interface->updateCooperateStatus(
          uuid, true, autoware::rtc_interface::State::SUCCEEDED,
          std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(), ros::Time::now());
      }
    }
  }

  void onExit() override
  {
    if (getCurrentStatus() == ModuleStatus::SUCCESS) updateRTCStatusForSuccess();
    SceneModuleCore::onExit();
  }

  void publishObjectsOfInterestMarker()
  {
    for (const auto & [module_name, interface] : marker_interfaces_) {
      if (interface) interface->publishMarkerArray();
    }
  }

  virtual visualization_msgs::MarkerArray getModuleVirtualWall() { return {}; }
  PoseWithDetailOpt getStopPose() const
  {
    if (!stop_pose_) {
      return {};
    }

    const auto & base_link2front = planner_data_->parameters.base_link2front;
    return PoseWithDetail(
      autoware_utils_geometry::calc_offset_pose(stop_pose_.value().pose, base_link2front, 0.0, 0.0),
      stop_pose_.value().detail);
  }

  PoseWithDetailOpt getSlowPose() const
  {
    if (!slow_pose_) {
      return {};
    }

    const auto & base_link2front = planner_data_->parameters.base_link2front;
    return PoseWithDetail(
      autoware_utils_geometry::calc_offset_pose(slow_pose_.value().pose, base_link2front, 0.0, 0.0),
      slow_pose_.value().detail);
  }

  PoseWithDetailOpt getDeadPose() const
  {
    if (!dead_pose_) {
      return {};
    }

    const auto & base_link2front = planner_data_->parameters.base_link2front;
    return PoseWithDetail(
      autoware_utils_geometry::calc_offset_pose(dead_pose_.value().pose, base_link2front, 0.0, 0.0),
      dead_pose_.value().detail);
  }

  void resetWallPoses() const
  {
    stop_pose_ = std::nullopt;
    slow_pose_ = std::nullopt;
    dead_pose_ = std::nullopt;
  }

protected:

  void refreshApprovalState() override
  {
    bool has_registered = false;
    bool has_approved = false;
    bool has_unapproved = false;
    for (const auto & [module_name, interface] : rtc_interfaces_) {
      if (!interface) continue;
      const auto & uuid = rtc_uuids_.at(module_name);
      if (!interface->isRegistered(uuid)) continue;
      has_registered = true;
      if (interface->isTerminated(uuid)) continue;
      if (interface->isActivated(uuid)) {
        has_approved = true;
      } else {
        has_unapproved = true;
      }
    }
    setApprovalState(ApprovalState{
      has_registered, has_approved, false,
      true, has_unapproved, !has_registered || has_approved});
  }

  // Preserve the source RTC approval policy for provider-bearing modules.
  // With no registered RTC provider, execution is autonomous; otherwise at
  // least one registered command must be activated.
  bool isActivated() const
  {
    if (rtc_interfaces_.empty()) return true;
    for (const auto & [module_name, interface] : rtc_interfaces_) {
      if (!interface) continue;
      const auto & uuid = rtc_uuids_.at(module_name);
      if (!interface->isRegistered(uuid)) continue;
      if (!interface->isTerminated(uuid) && interface->isActivated(uuid)) return true;
    }
    return false;
  }

  void setObjectsOfInterestData(
    const geometry_msgs::Pose & object_pose, const autoware_perception_msgs::Shape & object_shape,
    const autoware::objects_of_interest_marker_interface::ColorName & color_name)
  {
    for (const auto & [module_name, interface] : marker_interfaces_) {
      if (interface) interface->insertObjectData(object_pose, object_shape, color_name);
    }
  }

  void set_longitudinal_planning_factor(
    const autoware_internal_planning_msgs::PathWithLaneId & path)
  {
    if (!planning_factor_interface_) return;
    if (stop_pose_) {
      planning_factor_interface_->add(
        path.points, getEgoPose(), stop_pose_->pose,
        autoware::planning_factor_interface::PlanningFactor::STOP,
        autoware_internal_planning_msgs::SafetyFactorArray{}, true, 0.0, 0.0,
        stop_pose_->detail);
      return;
    }
    if (slow_pose_) {
      planning_factor_interface_->add(
        path.points, getEgoPose(), slow_pose_->pose,
        autoware::planning_factor_interface::PlanningFactor::SLOW_DOWN,
        autoware_internal_planning_msgs::SafetyFactorArray{}, true, 0.0, 0.0,
        slow_pose_->detail);
    }
  }

  void setDrivableLanes(const std::vector<DrivableLanes> & drivable_lanes)
  {
    drivable_lanes_marker_ =
      marker_utils::createDrivableLanesMarkerArray(drivable_lanes, "drivable_lanes");
  }

  const RTCInterfaceMap & rtcInterfaces() const { return rtc_interfaces_; }
  const MarkerInterfaceMap & markerInterfaces() const { return marker_interfaces_; }
  const unique_identifier_msgs::UUID & rtcUuid(const std::string & module_name) const
  {
    return rtc_uuids_.at(module_name);
  }
  const std::shared_ptr<autoware::planning_factor_interface::PlanningFactorInterface> & planningFactorInterface() const
  {
    return planning_factor_interface_;
  }
  // Provider state is intentionally protected for concrete BPP modules.  They
  // own module-specific RTC state transitions and marker insertion, while this
  // class retains construction, UUID generation and publication ownership.
  RTCInterfaceMap rtc_interfaces_;
  MarkerInterfaceMap marker_interfaces_;
  std::shared_ptr<autoware::planning_factor_interface::PlanningFactorInterface> planning_factor_interface_;
  std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_;
  std::unordered_map<std::string, unique_identifier_msgs::UUID> rtc_uuids_;
  // Source-compatible views of the provider state.  These aliases are not
  // copies: a concrete ROS1 module observes the manager-owned RTC/marker/UUID
  // lifecycle exactly once and cannot diverge from publication ownership.
  const RTCInterfaceMap & rtc_interface_ptr_map_;
  MarkerInterfaceMap & objects_of_interest_marker_interface_ptr_map_;
  std::unordered_map<std::string, unique_identifier_msgs::UUID> & uuid_map_;
  std::shared_ptr<Ros1SceneModuleClock> clock_;
  mutable PoseWithDetailOpt stop_pose_;
  mutable PoseWithDetailOpt slow_pose_;
  mutable PoseWithDetailOpt dead_pose_;
  mutable visualization_msgs::MarkerArray info_marker_;
  mutable visualization_msgs::MarkerArray debug_marker_;
  mutable visualization_msgs::MarkerArray drivable_lanes_marker_;
};
}  // namespace autoware::behavior_path_planner
