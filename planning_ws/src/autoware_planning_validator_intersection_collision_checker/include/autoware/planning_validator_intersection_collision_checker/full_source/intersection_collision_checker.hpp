// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__PLANNING_VALIDATOR_INTERSECTION_COLLISION_CHECKER__INTERSECTION_COLLISION_CHECKER_FULL_SOURCE_HPP_  // NOLINT
#define AUTOWARE__PLANNING_VALIDATOR_INTERSECTION_COLLISION_CHECKER__INTERSECTION_COLLISION_CHECKER_FULL_SOURCE_HPP_  // NOLINT

#include "autoware/planning_validator_intersection_collision_checker/full_source/types.hpp"

#include <autoware/planning_validator/full_source/plugin_interface.hpp>
#include <autoware/planning_validator/full_source/native_support.hpp>

#include <autoware_internal_planning_msgs/SafetyFactorArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::planning_validator::full_source
{
using sensor_msgs::PointCloud2;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using autoware_internal_planning_msgs::SafetyFactor;
using autoware_internal_planning_msgs::SafetyFactorArray;

class IntersectionCollisionChecker : public PluginInterface
{
public:
  void init(
    validator_native::Node & node, const std::string & name,
    const std::shared_ptr<PlanningValidatorContext> & context) override;
  void validate() override;
  void setup_diag() override;
  std::string get_module_name() const override { return module_name_; };

private:
  bool is_data_ready(std::string & msg);

  [[nodiscard]] bool is_safe(IntersectionDebugData & debug_data);

  [[nodiscard]] EgoTrajectory get_ego_trajectory() const;

  void get_lanelets(IntersectionDebugData & debug_data, const EgoTrajectory & ego_trajectory) const;

  [[nodiscard]] Direction get_turn_direction(
    const lanelet::ConstLanelets & trajectory_lanelets) const;

  void filter_pointcloud(
    PointCloud2::ConstPtr & input, PointCloud::Ptr & filtered_point_cloud,
    IntersectionDebugData & debug_data) const;

  void get_points_within(
    const PointCloud::Ptr & input, const BasicPolygon2d & polygon,
    const PointCloud::Ptr & output) const;

  void cluster_pointcloud(
    const PointCloud::Ptr & input, PointCloud::Ptr & output, IntersectionDebugData & debug_data) const;

  bool check_collision(
    IntersectionDebugData & debug_data, const PointCloud::Ptr & filtered_point_cloud,
    const ros::Time & time_stamp);

  std::optional<PCDObject> get_pcd_object(
    IntersectionDebugData & debug_data, const ros::Time & time_stamp,
    const PointCloud::Ptr & filtered_point_cloud, const TargetLanelet & target_lanelet) const;

  void update_tracked_object(PCDObject & object, const PCDObject & new_data) const;

  void publish_markers(const IntersectionDebugData & debug_data) const;

  void set_lanelets_debug_marker(const IntersectionDebugData & debug_data) const;

  void set_pcd_objects_debug_marker(const IntersectionDebugData & debug_data) const;

  void publish_planning_factor(const IntersectionDebugData & debug_data) const;

  void set_diag_status(
    DiagnosticStatusWrapper & stat, const bool & is_ok, const std::string & msg) const;

  void reset_data()
  {
    history_.clear();
    target_lanelets_map_.clear();
    last_valid_time_ = clock_->now();
  }

  std::unique_ptr<intersection_collision_checker_node::ParamListener> param_listener_;

  validator_native::Publisher<PointCloud2> pub_voxel_pointcloud_;
  validator_native::Publisher<PointCloud2> pub_cluster_pointcloud_;
  validator_native::Publisher<StringStamped> pub_string_;

  intersection_collision_checker_node::Params params_;

  PCDObjectsMap history_;
  mutable TargetLaneletsMap target_lanelets_map_;
  std::vector<lanelet::Id> collision_lanes_;
  ros::Time last_invalid_time_;
  ros::Time last_valid_time_;
};

}  // namespace autoware::planning_validator::full_source

// clang-format off
#endif  // AUTOWARE__PLANNING_VALIDATOR_INTERSECTION_COLLISION_CHECKER__INTERSECTION_COLLISION_CHECKER_FULL_SOURCE_HPP_  // NOLINT
// clang-format on
