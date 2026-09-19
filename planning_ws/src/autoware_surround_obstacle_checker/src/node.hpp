// Copyright 2020 Tier IV, Inc.
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

#ifndef NODE_HPP_
#define NODE_HPP_

#include "autoware_utils/ros/logger_level_configure.hpp"
#include "autoware_utils/ros/polling_subscriber.hpp"
#include "debug_marker.hpp"
#include "type_alias.hpp"
#include "types.hpp"

#include <autoware/motion_utils/vehicle/vehicle_state_checker.hpp>
#include "full_native_support.hpp"
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include "full_native_support.hpp"
#include <tf2/utils.h>

#include <autoware_internal_debug_msgs/Float64Stamped.h>
#include <autoware_internal_planning_msgs/VelocityLimit.h>
#include <autoware_internal_planning_msgs/VelocityLimitClearCommand.h>
#include <autoware_perception_msgs/PredictedObjects.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::surround_obstacle_checker
{

enum class State { PASS, STOP };

class SurroundObstacleCheckerNode : public full_native::Node
{
public:
  explicit SurroundObstacleCheckerNode(ros::NodeHandle private_node);

private:
  std::array<double, 3> getCheckDistances(const std::string & str_label) const;

  bool getUseDynamicObject() const;

  void onTimer();

  std::optional<StopObstacle> getNearestObstacle() const;

  std::optional<StopObstacle> getNearestObstacleByPointCloud() const;

  std::optional<StopObstacle> getNearestObstacleByDynamicObject() const;

  std::optional<geometry_msgs::TransformStamped> getTransform(
    const std::string & source, const std::string & target, const ros::Time & stamp,
    double duration_sec) const;

  auto isStopRequired(
    const bool is_obstacle_found, const bool is_vehicle_stopped, const State & state,
    const std::optional<ros::Time> & last_obstacle_found_time, const double time_threshold) const
    -> std::pair<bool, std::optional<ros::Time>>;

  // ros
  mutable tf2_ros::Buffer tf_buffer_{};
  mutable tf2_ros::TransformListener tf_listener_{tf_buffer_};
  ros::Timer timer_;

  // publisher and subscriber
  autoware_utils::InterProcessPollingSubscriber<nav_msgs::Odometry> sub_odometry_{
    this, "input/odometry"};
  autoware_utils::InterProcessPollingSubscriber<sensor_msgs::PointCloud2> sub_pointcloud_{
    this, "input/pointcloud", autoware_utils::single_depth_sensor_qos()};
  autoware_utils::InterProcessPollingSubscriber<PredictedObjects> sub_dynamic_objects_{
    this, "input/objects"};
  ros::Publisher pub_clear_velocity_limit_;
  ros::Publisher pub_velocity_limit_;
  ros::Publisher
    pub_processing_time_;

  // stop checker
  std::unique_ptr<VehicleStopChecker> vehicle_stop_checker_;

  // debug
  std::shared_ptr<SurroundObstacleCheckerDebugNode> debug_ptr_;

  // parameter
  std::shared_ptr<surround_obstacle_checker_node::ParamListener> param_listener_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

  // data
  nav_msgs::Odometry::ConstPtr odometry_ptr_;
  sensor_msgs::PointCloud2::ConstPtr pointcloud_ptr_;
  PredictedObjects::ConstPtr object_ptr_;

  // State Machine
  State state_ = State::PASS;
  std::optional<ros::Time> last_obstacle_found_time_;

  std::unique_ptr<autoware_utils::LoggerLevelConfigure> logger_configure_;

  std::unordered_map<int, std::string> label_map_;

public:
  friend class SurroundObstacleCheckerNodeTest;
};
}  // namespace autoware::surround_obstacle_checker

#endif  // NODE_HPP_
