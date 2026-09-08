// Copyright 2019 Autoware Foundation
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

#ifndef AUTOWARE__BEHAVIOR_VELOCITY_PLANNER__FULL_NODE_HPP_
#define AUTOWARE__BEHAVIOR_VELOCITY_PLANNER__FULL_NODE_HPP_

#include "autoware/behavior_velocity_planner/planner_manager.hpp"

#include <autoware/behavior_velocity_planner_common/planner_data.hpp>
#include <autoware_utils_debug/published_time_publisher.hpp>
#include <autoware_utils_logging/logger_level_configure.hpp>
#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <autoware_utils_system/stop_watch.hpp>
#include "autoware/behavior_velocity_planner/full_native_support.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <autoware_internal_debug_msgs/Float64Stamped.h>
#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <autoware_internal_planning_msgs/VelocityLimit.h>
#include <autoware_internal_planning_msgs/LoadPlugin.h>
#include <autoware_internal_planning_msgs/UnloadPlugin.h>
#include <autoware_map_msgs/LaneletMapBin.h>
#include <autoware_perception_msgs/PredictedObjects.h>
#include <autoware_planning_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner
{
using autoware_internal_debug_msgs::Float64Stamped;
using autoware_internal_planning_msgs::VelocityLimit;
using autoware_internal_planning_msgs::LoadPlugin;
using autoware_internal_planning_msgs::UnloadPlugin;
using autoware_map_msgs::LaneletMapBin;

class FullBehaviorVelocityPlannerNode : public bvp_full_native::Node
{
public:
  explicit FullBehaviorVelocityPlannerNode(ros::NodeHandle private_node);

private:
  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // subscriber
  ros::Subscriber trigger_sub_path_with_lane_id_;

  // polling subscribers
  autoware_utils_rclcpp::InterProcessPollingSubscriber<
    autoware_perception_msgs::PredictedObjects>
    sub_predicted_objects_{this, "input/dynamic_objects"};

  autoware_utils_rclcpp::InterProcessPollingSubscriber<sensor_msgs::PointCloud2>
    sub_no_ground_pointcloud_{
      this, "input/no_ground_pointcloud", 1U};

  autoware_utils_rclcpp::InterProcessPollingSubscriber<nav_msgs::Odometry>
    sub_vehicle_odometry_{this, "input/vehicle_odometry"};

  autoware_utils_rclcpp::InterProcessPollingSubscriber<
    geometry_msgs::AccelWithCovarianceStamped>
    sub_acceleration_{this, "input/accel"};

  autoware_utils_rclcpp::InterProcessPollingSubscriber<
    autoware_perception_msgs::TrafficLightGroupArray>
    sub_traffic_signals_{this, "input/traffic_signals"};

  autoware_utils_rclcpp::InterProcessPollingSubscriber<nav_msgs::OccupancyGrid>
    sub_occupancy_grid_{this, "input/occupancy_grid"};

  autoware_utils_rclcpp::InterProcessPollingSubscriber<
    LaneletMapBin, autoware_utils_rclcpp::polling_policy::Newest>
    sub_lanelet_map_{this, "input/vector_map", 1U};

  autoware_utils_rclcpp::InterProcessPollingSubscriber<VelocityLimit> sub_external_velocity_limit_{
    this, "input/external_velocity_limit_mps", 1U};

  void onTrigger(
    const autoware_internal_planning_msgs::PathWithLaneId::ConstPtr input_path_msg);

  void onParam();

  bool processNoGroundPointCloud(const sensor_msgs::PointCloud2::ConstPtr msg);
  void processOdometry(const nav_msgs::Odometry::ConstPtr msg);
  void processTrafficSignals(
    const autoware_perception_msgs::TrafficLightGroupArray::ConstPtr msg);
  bool processData(bvp_full_native::Clock clock);

  // publisher
  bvp_full_native::Publisher<autoware_planning_msgs::Path> path_pub_;
  bvp_full_native::Publisher<visualization_msgs::MarkerArray> debug_viz_pub_;
  bvp_full_native::Publisher<Float64Stamped> processing_time_publisher_;

  void publishDebugMarker(const autoware_planning_msgs::Path & path);
  void publishProcessingTime();

  //  parameter
  double forward_path_length_;
  double backward_path_length_;
  double behavior_output_path_interval_;

  // member
  PlannerData planner_data_;
  BehaviorVelocityPlannerManager planner_manager_;
  bool is_driving_forward_{true};
  autoware_utils_system::StopWatch<std::chrono::milliseconds> stop_watch_;

  ros::ServiceServer srv_load_plugin_;
  ros::ServiceServer srv_unload_plugin_;
  bool onUnloadPlugin(UnloadPlugin::Request & request, UnloadPlugin::Response & response);
  bool onLoadPlugin(LoadPlugin::Request & request, LoadPlugin::Response & response);

  // mutex for planner_data_
  std::mutex mutex_;

  // function
  bool isDataReady(bvp_full_native::Clock clock);
  autoware_planning_msgs::Path generatePath(
    const autoware_internal_planning_msgs::PathWithLaneId::ConstPtr input_path_msg,
    const PlannerData & planner_data);

  std::unique_ptr<autoware_utils_logging::LoggerLevelConfigure> logger_configure_;

  std::unique_ptr<autoware_utils_debug::PublishedTimePublisher> published_time_publisher_;

  static constexpr int logger_throttle_interval = 3000;
};
}  // namespace autoware::behavior_velocity_planner

#endif  // AUTOWARE__BEHAVIOR_VELOCITY_PLANNER__FULL_NODE_HPP_
