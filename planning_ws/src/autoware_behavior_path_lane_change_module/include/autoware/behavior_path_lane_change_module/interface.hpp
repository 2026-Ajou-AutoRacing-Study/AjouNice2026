// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__INTERFACE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__INTERFACE_HPP_

#include "autoware/behavior_path_lane_change_module/base_class.hpp"
#include "autoware/behavior_path_lane_change_module/scene.hpp"
#include "autoware/behavior_path_lane_change_module/structs/data.hpp"
#include "autoware/behavior_path_lane_change_module/structs/path.hpp"
#include "autoware/behavior_path_planner_common/interface/scene_module_interface.hpp"
#include "autoware/behavior_path_planner_common/interface/scene_module_visitor.hpp"
#include "autoware/behavior_path_planner_common/turn_signal_decider.hpp"
#include "autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"

#include <autoware_utils/system/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>

#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <memory>
#include <any>
#include <string>
#include <unordered_map>
#include <utility>

namespace autoware::behavior_path_planner
{
using autoware::objects_of_interest_marker_interface::ColorName;
using autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface;
using autoware::planning_factor_interface::PlanningFactorInterface;
using autoware::rtc_interface::RTCInterface;
using autoware::rtc_interface::State;
using autoware_internal_planning_msgs::PathWithLaneId;
using autoware_internal_planning_msgs::PlanningFactor;
using autoware_internal_planning_msgs::SafetyFactorArray;
using autoware_adapi_v1_msgs::OperationModeState;
using geometry_msgs::Pose;
using geometry_msgs::Twist;
using visualization_msgs::MarkerArray;

class LaneChangeInterface : public SceneModuleInterface
{
public:
  LaneChangeInterface(
    const std::string & name, ros::NodeHandle & node, std::shared_ptr<LaneChangeParameters> parameters,
    const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
    std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
      objects_of_interest_marker_interface_ptr_map,
    const std::shared_ptr<PlanningFactorInterface> & planning_factor_interface,
    std::unique_ptr<LaneChangeBase> && module_type);

  LaneChangeInterface(const LaneChangeInterface &) = delete;
  LaneChangeInterface(LaneChangeInterface &&) = delete;
  LaneChangeInterface & operator=(const LaneChangeInterface &) = delete;
  LaneChangeInterface & operator=(LaneChangeInterface &&) = delete;
  ~LaneChangeInterface() override = default;

  void processOnExit() override;

  bool isExecutionRequested() const override;

  bool isExecutionReady() const override;

  // This releases only Cruise suppression; the approved LC remains RUNNING.
  bool isObstacleCruiseAllowed() const;

  bool isCurrentRouteLaneletToBeReset() const
  {
    return getCurrentStatus() == ModuleStatus::SUCCESS;
  }

  void updateData() override;

  void postProcess() override;

  BehaviorModuleOutput plan() override;

  BehaviorModuleOutput planWaitingApproval() override;

  CandidateOutput planCandidate() const override;

  void acceptVisitor(
    [[maybe_unused]] const std::shared_ptr<SceneModuleVisitor> & visitor) const
  {
  }

  void updateModuleParams(const std::any & parameters);

  void setData(const std::shared_ptr<const PlannerData> & data) override;

  MarkerArray getModuleVirtualWall();

protected:
  using SceneModuleInterface::updateRTCStatus;

  std::shared_ptr<LaneChangeParameters> parameters_;

  std::unique_ptr<LaneChangeBase> module_type_;

  PathSafetyStatus post_process_safety_status_;

  bool canTransitSuccessState() override;

  bool canTransitFailureState() override;

  ModuleStatus setInitState() const override { return ModuleStatus::WAITING_APPROVAL; };

  void updateRTCStatus(
    const double start_distance, const double finish_distance, const bool safe,
    const uint8_t & state)
  {
    autoware_utils::ScopedTimeTrack st(__func__, *getTimeKeeper());
    for (const auto & [module_name, ptr] : rtcInterfaces()) {
      if (ptr) {
        ptr->updateCooperateStatus(
          rtcUuid(module_name), safe, state, start_distance, finish_distance, ros::Time::now());
      }
    }
  }

  std::pair<LaneChangeStates, std::string_view> check_transit_failure();

  void updateDebugMarker() const;

  void updateSteeringFactorPtr(const BehaviorModuleOutput & output);

  void updateSteeringFactorPtr(
    const CandidateOutput & output, const LaneChangePath & selected_path) const;

  bool is_rtc_force_activated() const
  {
    return std::any_of(rtc_interfaces_.begin(), rtc_interfaces_.end(), [&](const auto & rtc) {
      return rtc.second && rtc.second->isForceActivated(rtcUuid(rtc.first));
    });
  }

  bool is_rtc_force_deactivated() const
  {
    return std::any_of(rtc_interfaces_.begin(), rtc_interfaces_.end(), [&](const auto & rtc) {
      return rtc.second && rtc.second->isForceDeactivated(rtcUuid(rtc.first));
    });
  }

  void set_longitudinal_planning_factor(const PathWithLaneId & path)
  {
    if (stop_pose_ && planning_factor_interface_) {
      planning_factor_interface_->add(
        path.points, getEgoPose(), stop_pose_->pose, PlanningFactor::STOP, SafetyFactorArray{},
        true, 0.0, 0.0, stop_pose_->detail);
    }
  }

  mutable MarkerArray virtual_wall_marker_;

  void clearAbortApproval() { is_abort_path_approved_ = false; }

  bool is_abort_path_approved_{false};

  bool is_abort_approval_requested_{false};

  mutable lane_change::InterfaceDebug interface_debug_;
};
}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__INTERFACE_HPP_
