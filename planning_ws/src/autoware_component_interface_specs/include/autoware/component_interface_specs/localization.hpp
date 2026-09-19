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

#ifndef AUTOWARE__COMPONENT_INTERFACE_SPECS__LOCALIZATION_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_SPECS__LOCALIZATION_HPP_

#include <autoware/component_interface_specs/utils.hpp>

#include <autoware_adapi_v1_msgs/LocalizationInitializationState.h>
#include <autoware_localization_msgs/InitializeLocalization.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

namespace autoware::component_interface_specs::localization
{

struct Initialize
{
  using Service = autoware_localization_msgs::InitializeLocalization;
  static constexpr char name[] = "/localization/initialize";
};

struct InitializationState
{
  using Message = autoware_adapi_v1_msgs::LocalizationInitializationState;
  static constexpr char name[] = "/localization/initialization_state";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = QosReliability::Reliable;
  static constexpr auto durability = QosDurability::TransientLocal;
};

struct KinematicState
{
  using Message = nav_msgs::Odometry;
  static constexpr char name[] = "/localization/kinematic_state";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = QosReliability::Reliable;
  static constexpr auto durability = QosDurability::Volatile;
};

struct Acceleration
{
  using Message = geometry_msgs::AccelWithCovarianceStamped;
  static constexpr char name[] = "/localization/acceleration";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = QosReliability::Reliable;
  static constexpr auto durability = QosDurability::Volatile;
};

}  // namespace autoware::component_interface_specs::localization

#endif  // AUTOWARE__COMPONENT_INTERFACE_SPECS__LOCALIZATION_HPP_
