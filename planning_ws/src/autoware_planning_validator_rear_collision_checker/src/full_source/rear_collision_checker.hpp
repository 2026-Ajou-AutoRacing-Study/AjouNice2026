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

#ifndef REAR_COLLISION_CHECKER_HPP_
#define REAR_COLLISION_CHECKER_HPP_

#include "structs.hpp"

#include <autoware/planning_validator/full_source/plugin_interface.hpp>
#include <autoware_utils/system/time_keeper.hpp>
#include <autoware/planning_validator/full_source/native_support.hpp>

#include <map>
#include <memory>
#include <string>

namespace autoware::planning_validator::full_source
{

class RearCollisionChecker : public PluginInterface
{
public:
  void init(
    validator_native::Node & node, const std::string & name,
    const std::shared_ptr<PlanningValidatorContext> & context) override;
  void validate() override;
  void setup_diag() override;
  std::string get_module_name() const override { return module_name_; };

private:
  void fill_velocity(PointCloudObject & pointcloud_object);

  auto filter_pointcloud(RearDebugData & debug) const -> PointCloud::Ptr;

  auto get_clustered_pointcloud(const PointCloud::Ptr in, RearDebugData & debug) const
    -> PointCloud::Ptr;

  auto get_pointcloud_object(
    const ros::Time & now, const PointCloud::Ptr & pointcloud_ptr,
    const DetectionAreas & detection_areas, RearDebugData & debug) -> std::optional<PointCloudObject>;

  auto get_pointcloud_objects(
    const std::function<std::pair<double, double>()> & func_range_calculation,
    const std::function<PointCloudObjects(const double, const double)> & func_object_filtering,
    const std::function<void(PointCloudObjects &)> & func_safety_check) -> PointCloudObjects;

  auto get_pointcloud_objects_on_adjacent_lane(
    const lanelet::ConstLanelets & current_lanes, const Behavior & shift_behavior,
    const double forward_distance, const double backward_distance,
    const PointCloud::Ptr & obstacle_pointcloud, RearDebugData & debug) -> PointCloudObjects;

  auto get_pointcloud_objects_at_blind_spot(
    const lanelet::ConstLanelets & current_lanes, const Behavior & turn_behavior,
    const double forward_distance, const double backward_distance,
    const PointCloud::Ptr & obstacle_pointcloud, RearDebugData & debug) -> PointCloudObjects;

  bool is_safe(const PointCloudObjects & objects, RearDebugData & debug) const;

  bool is_safe(RearDebugData & debug);

  void post_process();

  void publish_marker(const RearDebugData & debug) const;

  void publish_planning_factor(const RearDebugData & debug) const;

  void set_diag_status(
    DiagnosticStatusWrapper & stat, const bool & is_ok, const std::string & msg) const;

  validator_native::Publisher<PointCloud2> pub_voxel_pointcloud_;

  validator_native::Publisher<PointCloud2> pub_cluster_pointcloud_;

  validator_native::Publisher<StringStamped> pub_string_;

  validator_native::Publisher<autoware_utils::ProcessingTimeDetail>
    pub_debug_processing_time_detail_;

  ros::Time last_safe_time_;

  ros::Time last_unsafe_time_;

  std::unique_ptr<rear_collision_checker_node::ParamListener> param_listener_;

  std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_;

  std::map<lanelet::Id, PointCloudObject> history_;
};

}  // namespace autoware::planning_validator::full_source

#endif  // REAR_COLLISION_CHECKER_HPP_
