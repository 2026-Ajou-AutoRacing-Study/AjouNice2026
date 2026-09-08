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

#ifndef AUTOWARE__COMPONENT_INTERFACE_SPECS__VEHICLE_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_SPECS__VEHICLE_HPP_

#include "utils.hpp"

#include <autoware/component_interface_specs/utils.hpp>

#include <autoware_vehicle_msgs/GearReport.h>
#include <autoware_vehicle_msgs/HazardLightsReport.h>
#include <autoware_vehicle_msgs/SteeringReport.h>
#include <autoware_vehicle_msgs/TurnIndicatorsReport.h>

namespace autoware::component_interface_specs::vehicle
{

struct SteeringStatus
{
  using Message = autoware_vehicle_msgs::SteeringReport;
  static constexpr char name[] = "/vehicle/status/steering_status";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = QosReliability::Reliable;
  static constexpr auto durability = QosDurability::Volatile;
};

struct GearStatus
{
  using Message = autoware_vehicle_msgs::GearReport;
  static constexpr char name[] = "/vehicle/status/gear_status";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = QosReliability::Reliable;
  static constexpr auto durability = QosDurability::Volatile;
};

struct TurnIndicatorStatus
{
  using Message = autoware_vehicle_msgs::TurnIndicatorsReport;
  static constexpr char name[] = "/vehicle/status/turn_indicators_status";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = QosReliability::Reliable;
  static constexpr auto durability = QosDurability::Volatile;
};

struct HazardLightStatus
{
  using Message = autoware_vehicle_msgs::HazardLightsReport;
  static constexpr char name[] = "/vehicle/status/hazard_lights_status";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = QosReliability::Reliable;
  static constexpr auto durability = QosDurability::Volatile;
};

}  // namespace autoware::component_interface_specs::vehicle

#endif  // AUTOWARE__COMPONENT_INTERFACE_SPECS__VEHICLE_HPP_
