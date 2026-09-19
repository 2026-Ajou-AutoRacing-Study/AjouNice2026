// Copyright 2022 Tier IV, Inc.
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

#ifndef AUTOWARE__RTC_INTERFACE__RTC_INTERFACE_HPP_
#define AUTOWARE__RTC_INTERFACE__RTC_INTERFACE_HPP_

#include <ros/ros.h>

#include <tier4_rtc_msgs/AutoMode.h>
#include <tier4_rtc_msgs/AutoModeStatus.h>
#include <tier4_rtc_msgs/Command.h>
#include <tier4_rtc_msgs/CooperateCommand.h>
#include <tier4_rtc_msgs/CooperateCommands.h>
#include <tier4_rtc_msgs/CooperateResponse.h>
#include <tier4_rtc_msgs/CooperateStatus.h>
#include <tier4_rtc_msgs/CooperateStatusArray.h>
#include <tier4_rtc_msgs/Module.h>
#include <tier4_rtc_msgs/State.h>
#include <unique_identifier_msgs/UUID.h>

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace autoware::rtc_interface
{
using tier4_rtc_msgs::AutoMode;
using tier4_rtc_msgs::AutoModeStatus;
using tier4_rtc_msgs::Command;
using tier4_rtc_msgs::CooperateCommand;
using tier4_rtc_msgs::CooperateCommands;
using tier4_rtc_msgs::CooperateResponse;
using tier4_rtc_msgs::CooperateStatus;
using tier4_rtc_msgs::CooperateStatusArray;
using tier4_rtc_msgs::Module;
using tier4_rtc_msgs::State;
using unique_identifier_msgs::UUID;

class RTCInterface
{
public:
  RTCInterface(ros::NodeHandle * node, const std::string & name, const bool enable_rtc = true);
  void publishCooperateStatus(const ros::Time & stamp);
  /// @brief update the cooperate status of the module identified by the given UUID
  /// @param[in] uuid unique ID of the module
  /// @param[in] safe new value for the "safe" field
  /// @param[in] state new value for the "state" field
  /// @param[in] start_distance new value for the "start_distance" field
  /// @param[in] finish_distance new value for the "finish_distance" field
  /// @param[in] stamp new value for the "stamp" field
  /// @param[in] requested new value for the "requested" field (default to false)
  /// @param[in] override_rtc_auto_mode optional value of the "auto_mode" field (if not set, the
  /// "auto_mode" is only true if enable_rtc was false at initialization)
  void updateCooperateStatus(
    const UUID & uuid, const bool safe, const uint8_t state, const double start_distance,
    const double finish_distance, const ros::Time & stamp, const bool requested = false,
    const std::optional<bool> & override_rtc_auto_mode = std::nullopt);
  void removeCooperateStatus(const UUID & uuid);
  void removeExpiredCooperateStatus();
  void clearCooperateStatus();
  bool isActivated(const UUID & uuid) const;
  bool isForceActivated(const UUID & uuid) const;
  bool isForceDeactivated(const UUID & UUID) const;
  bool isRegistered(const UUID & uuid) const;
  bool isRTCEnabled(const UUID & uuid) const;
  bool isTerminated(const UUID & uuid) const;
  void lockCommandUpdate();
  void unlockCommandUpdate();
  void print() const;

private:
  bool onCooperateCommandService(
    CooperateCommands::Request & request, CooperateCommands::Response & responses);
  bool onAutoModeService(AutoMode::Request & request, AutoMode::Response & response);
  void onTimer(const ros::TimerEvent &);
  std::vector<CooperateResponse> validateCooperateCommands(
    const std::vector<CooperateCommand> & commands);
  void updateCooperateCommandStatus(const std::vector<CooperateCommand> & commands);
  void removeStoredCommand(const UUID & uuid);
  bool isLocked() const;

  ros::Publisher pub_statuses_;
  ros::Publisher pub_auto_mode_status_;
  ros::ServiceServer srv_commands_;
  ros::ServiceServer srv_auto_mode_;
  ros::Timer timer_;
  std::string logger_name_;

  Module module_;
  CooperateStatusArray registered_status_;
  std::vector<CooperateCommand> stored_commands_;
  bool is_auto_mode_enabled_;
  bool is_locked_;

  std::string cooperate_status_namespace_ = "/planning/cooperate_status";
  std::string auto_mode_status_namespace_ = "/planning/auto_mode_status";
  std::string cooperate_commands_namespace_ = "/planning/cooperate_commands";
  std::string enable_auto_mode_namespace_ = "/planning/enable_auto_mode";

  mutable std::mutex mutex_;

public:
  friend class RTCInterfaceTest;
};

}  // namespace autoware::rtc_interface

#endif  // AUTOWARE__RTC_INTERFACE__RTC_INTERFACE_HPP_
