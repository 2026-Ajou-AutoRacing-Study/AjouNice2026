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

#ifndef INTERFACE_HPP_
#define INTERFACE_HPP_

#include "autoware/behavior_path_lane_change_module/interface.hpp"
#include "data_structs.hpp"
#include "scene.hpp"

#include <ros/ros.h>

#include <memory>
#include <string>
#include <unordered_map>

namespace autoware::behavior_path_planner
{
using autoware::behavior_path_planner::LaneChangeInterface;
using autoware::behavior_path_planner::ObjectsOfInterestMarkerInterface;
using autoware::behavior_path_planner::RTCInterface;

class AvoidanceByLaneChangeInterface : public LaneChangeInterface
{
public:
  AvoidanceByLaneChangeInterface(
    const std::string & name, ros::NodeHandle & node,
    const std::shared_ptr<LaneChangeParameters> & parameters,
    const std::shared_ptr<AvoidanceByLCParameters> & avoidance_by_lane_change_parameters,
    const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
    std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
      objects_of_interest_marker_interface_ptr_map,
    const std::shared_ptr<PlanningFactorInterface> & planning_factor_interface);

  bool isExecutionRequested() const override;

  void processOnEntry() override;

protected:
  // The ROS1 LaneChange provider exposes its generic RTC update with explicit
  // safety/state arguments.  Keep this direction-aware helper local to this
  // module rather than pretending to override the ROS2 two-argument hook.
  void updateRTCStatus(const double start_distance, const double finish_distance);
};
}  // namespace autoware::behavior_path_planner

#endif  // INTERFACE_HPP_
