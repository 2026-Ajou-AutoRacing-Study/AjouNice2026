// Copyright 2021 Tier IV, Inc.
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

#ifndef SCENE_OCCLUSION_SPOT_HPP_
#define SCENE_OCCLUSION_SPOT_HPP_

#include "occlusion_spot_utils.hpp"

#include <autoware/behavior_velocity_planner_common/scene_module_interface.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <autoware_utils/system/time_keeper.hpp>
#include <autoware/behavior_velocity_occlusion_spot_module/ros1_support.hpp>

#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <autoware_perception_msgs/PredictedObject.h>
#include <autoware_perception_msgs/PredictedObjects.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>

#include <boost/optional.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner
{
class OcclusionSpotModule : public SceneModuleInterface
{
  using PlannerParam = occlusion_spot_utils::PlannerParam;
  using DebugData = occlusion_spot_utils::DebugData;

public:
  OcclusionSpotModule(
    const int64_t module_id, const std::shared_ptr<const PlannerData> & planner_data,
    const PlannerParam & planner_param, ros::NodeHandle & node, const std::string & logger,
    const std::shared_ptr<OcclusionSpotRos1Clock> clock,
    const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
    const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
      planning_factor_interface);

  /**
   * @brief plan occlusion spot velocity at unknown area in occupancy grid
   */
  bool modifyPathVelocity(PathWithLaneId * path) override;

  visualization_msgs::MarkerArray createDebugMarkerArray() override;
  autoware::motion_utils::VirtualWalls createVirtualWalls() override;

private:
  std::string logger_;
  std::shared_ptr<OcclusionSpotRos1Clock> clock_{std::make_shared<OcclusionSpotRos1Clock>()};
  // Parameter
  PlannerParam param_;
  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch_;
  std::vector<lanelet::BasicPolygon2d> partition_lanelets_;

protected:
  int64_t module_id_{};

  // Debug
  mutable DebugData debug_data_;
};
}  // namespace autoware::behavior_velocity_planner

#endif  // SCENE_OCCLUSION_SPOT_HPP_
