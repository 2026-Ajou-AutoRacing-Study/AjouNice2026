// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__COMPONENT_INTERFACE_SPECS__SYSTEM_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_SPECS__SYSTEM_HPP_


#include <autoware/component_interface_specs/utils.hpp>

#include <autoware_adapi_v1_msgs/MrmState.h>
#include <autoware_adapi_v1_msgs/OperationModeState.h>
#include <autoware_system_msgs/ChangeAutowareControl.h>
#include <autoware_system_msgs/ChangeOperationMode.h>

namespace autoware::component_interface_specs::system
{

struct ChangeAutowareControl
{
  using Service = autoware_system_msgs::ChangeAutowareControl;
  static constexpr char name[] = "/system/operation_mode/change_autoware_control";
};

struct ChangeOperationMode
{
  using Service = autoware_system_msgs::ChangeOperationMode;
  static constexpr char name[] = "/system/operation_mode/change_operation_mode";
};

struct OperationModeState
{
  using Message = autoware_adapi_v1_msgs::OperationModeState;
  static constexpr char name[] = "/system/operation_mode/state";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = QosReliability::Reliable;
  static constexpr auto durability = QosDurability::TransientLocal;
};

}  // namespace autoware::component_interface_specs::system

#endif  // AUTOWARE__COMPONENT_INTERFACE_SPECS__SYSTEM_HPP_
