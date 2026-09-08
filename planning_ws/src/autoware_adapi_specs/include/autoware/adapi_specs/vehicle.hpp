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

#ifndef AUTOWARE__ADAPI_SPECS__VEHICLE_HPP_
#define AUTOWARE__ADAPI_SPECS__VEHICLE_HPP_

#include <autoware/adapi_specs/qos.hpp>

#include <autoware_adapi_v1_msgs/DoorStatusArray.h>
#include <autoware_adapi_v1_msgs/VehicleKinematics.h>
#include <autoware_adapi_v1_msgs/VehicleMetrics.h>
#include <autoware_adapi_v1_msgs/VehicleStatus.h>
#include <autoware_adapi_v1_msgs/GetDoorLayout.h>
#include <autoware_adapi_v1_msgs/GetVehicleDimensions.h>
#include <autoware_adapi_v1_msgs/SetDoorCommand.h>

namespace autoware::adapi_specs::vehicle
{

struct VehicleKinematics
{
  using Message = autoware_adapi_v1_msgs::VehicleKinematics;
  static constexpr char name[] = "/api/vehicle/kinematics";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

struct VehicleStatus
{
  using Message = autoware_adapi_v1_msgs::VehicleStatus;
  static constexpr char name[] = "/api/vehicle/status";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

struct VehicleMetrics
{
  using Message = autoware_adapi_v1_msgs::VehicleMetrics;
  static constexpr char name[] = "/api/vehicle/metrics";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

struct Dimensions
{
  using Service = autoware_adapi_v1_msgs::GetVehicleDimensions;
  static constexpr char name[] = "/api/vehicle/dimensions";
};

struct DoorCommand
{
  using Service = autoware_adapi_v1_msgs::SetDoorCommand;
  static constexpr char name[] = "/api/vehicle/doors/command";
};

struct DoorLayout
{
  using Service = autoware_adapi_v1_msgs::GetDoorLayout;
  static constexpr char name[] = "/api/vehicle/doors/layout";
};

struct DoorStatus
{
  using Message = autoware_adapi_v1_msgs::DoorStatusArray;
  static constexpr char name[] = "/api/vehicle/doors/status";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
};

}  // namespace autoware::adapi_specs::vehicle

#endif  // AUTOWARE__ADAPI_SPECS__VEHICLE_HPP_
