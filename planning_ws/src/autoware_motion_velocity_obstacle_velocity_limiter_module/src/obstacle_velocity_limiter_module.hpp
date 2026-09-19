// Copyright 2022-2024 TIER IV, Inc.
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

#ifndef OBSTACLE_VELOCITY_LIMITER_MODULE_HPP_
#define OBSTACLE_VELOCITY_LIMITER_MODULE_HPP_

#include "parameters.hpp"

#include <autoware/motion_velocity_planner_common/plugin_module_interface.hpp>
#include <autoware/motion_velocity_planner_common/velocity_planning_result.hpp>
#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <autoware_utils/ros/processing_time_publisher.hpp>
#include <ros/ros.h>

#include <autoware_map_msgs/LaneletMapBin.h>
#include <autoware_perception_msgs/PredictedObjects.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/optional.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::motion_velocity_planner
{

class ObstacleVelocityLimiterModule : public PluginModuleInterface
{
public:
  void init(ros::NodeHandle & node, const std::string & module_name) override;
  void updateParameters(const ParameterMap & parameters) override;
  VelocityPlanningResult plan(
    const std::vector<autoware_planning_msgs::TrajectoryPoint> & raw_trajectory_points,
    const std::vector<autoware_planning_msgs::TrajectoryPoint> & smoothed_trajectory_points,
    const std::shared_ptr<const PlannerData> & planner_data) override;
  std::string getModuleName() const override { return module_name_; }
  std::string getShortModuleName() const override { return "obstacle_velocity_limiter"; }
  RequiredSubscriptionInfo getRequiredSubscriptions() const override
  {
    RequiredSubscriptionInfo required_subscription_info;
    required_subscription_info.predicted_objects = true;
    required_subscription_info.no_ground_pointcloud = true;
    required_subscription_info.occupancy_grid_map = true;
    required_subscription_info.lanelet_map = true;
    return required_subscription_info;
  }

private:
  inline static const std::string ns_ = "obstacle_velocity_limiter";
  std::string module_name_;
  ros::Publisher debug_trajectory_publisher_;
  std::shared_ptr<ros::NodeHandle> node_owner_;
  std::shared_ptr<autoware_utils::ProcessingTimePublisher> processing_diag_publisher_;
  autoware::motion_utils::VirtualWallMarkerCreator virtual_wall_marker_creator_;
  std::optional<geometry_msgs::Point> prev_inserted_point_;

  // parameters
  obstacle_velocity_limiter::PreprocessingParameters preprocessing_params_;
  obstacle_velocity_limiter::ProjectionParameters projection_params_;
  obstacle_velocity_limiter::ObstacleParameters obstacle_params_;
  obstacle_velocity_limiter::VelocityParameters velocity_params_;
  double distance_buffer_{};
  double vehicle_lateral_offset_{};
  double vehicle_front_offset_{};
};
}  // namespace autoware::motion_velocity_planner

#endif  // OBSTACLE_VELOCITY_LIMITER_MODULE_HPP_
