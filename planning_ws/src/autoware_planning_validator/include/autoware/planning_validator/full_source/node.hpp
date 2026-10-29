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

#ifndef AUTOWARE__PLANNING_VALIDATOR__NODE_FULL_SOURCE_HPP_
#define AUTOWARE__PLANNING_VALIDATOR__NODE_FULL_SOURCE_HPP_

#include "autoware/planning_validator/full_source/debug_marker.hpp"
#include "autoware/planning_validator/full_source/manager.hpp"
#include "autoware/planning_validator/full_source/types.hpp"
#include "autoware_planning_validator/PlanningValidatorStatus.h"

#include <autoware_utils/ros/logger_level_configure.hpp>
#include <autoware_utils/ros/parameter.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils/ros/published_time_publisher.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <diagnostic_updater/diagnostic_updater.h>
#include <autoware/planning_validator/full_source/native_support.hpp>

#include <autoware_adapi_v1_msgs/OperationModeState.h>
#include <autoware_internal_debug_msgs/Float64Stamped.h>
#include <autoware_map_msgs/LaneletMapBin.h>
#include <autoware_planning_msgs/LaneletRoute.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <autoware_planning_msgs/StaticObstacleScene.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <autoware_perception_msgs/PredictedObjects.h>
#include <sensor_msgs/PointCloud2.h>

#include <ros/callback_queue.h>
#include <memory>
#include <string>

namespace autoware::planning_validator::full_source
{
using autoware_adapi_v1_msgs::OperationModeState;
using autoware_internal_debug_msgs::Float64Stamped;
using autoware_map_msgs::LaneletMapBin;
using autoware_planning_msgs::LaneletRoute;
using autoware_planning_msgs::Trajectory;
using autoware_planning_msgs::TrajectoryPoint;
using autoware_planning_validator::PlanningValidatorStatus;
using autoware_utils::StopWatch;
using diagnostic_updater::DiagnosticStatusWrapper;
using diagnostic_updater::Updater;
using geometry_msgs::AccelWithCovarianceStamped;
using nav_msgs::Odometry;
using sensor_msgs::PointCloud2;

class PlanningValidatorNode : public validator_native::Node
{
public:
  explicit PlanningValidatorNode(const ros::NodeHandle & options);

private:
  void onTrajectory(const Trajectory::ConstPtr & traj_msg);
  void setupParameters();
  void setData(const Trajectory::ConstPtr & traj_msg);
  bool isDataReady();

  void validate(const std::shared_ptr<const PlanningValidatorData> & data);

  void publishProcessingTime(const double processing_time_ms);
  void publishTrajectory();
  void onStaticScene(const autoware_planning_msgs::StaticObstacleScene::ConstPtr & scene);
  bool guardStaticScene(Trajectory & trajectory);
  void publishSceneCheckedTrajectory(Trajectory trajectory);
  void publishDebugInfo();
  void displayStatus();

  // subscriber
  ros::Subscriber sub_trajectory_;
  ros::Subscriber recovery_objects_sub_;
  autoware_perception_msgs::PredictedObjects::ConstPtr recovery_objects_;
  ros::CallbackQueue static_scene_queue_;
  ros::WallTimer static_scene_timer_;
  ros::Subscriber sub_static_scene_;
  autoware_planning_msgs::StaticObstacleScene::ConstPtr static_scene_;
  Trajectory::ConstPtr last_published_trajectory_;
  Odometry::ConstPtr scene_check_odometry_;
  uint32_t published_trajectory_sequence_{0};
  bool enable_static_scene_guard_{true};
  double static_scene_deceleration_{7.0};
  autoware_utils::InterProcessPollingSubscriber<
    LaneletRoute, autoware_utils::polling_policy::Newest>
    sub_route_{this, "input/route", 1};
  autoware_utils::InterProcessPollingSubscriber<
    LaneletMapBin, autoware_utils::polling_policy::Newest>
    sub_lanelet_map_bin_{this, "input/lanelet_map_bin", 1};
  autoware_utils::InterProcessPollingSubscriber<PointCloud2> sub_pointcloud_{
    this, "input/pointcloud", 1};
  autoware_utils::InterProcessPollingSubscriber<Odometry> sub_kinematics_{
    this, "input/kinematics"};
  autoware_utils::InterProcessPollingSubscriber<AccelWithCovarianceStamped> sub_acceleration_{
    this, "input/acceleration"};
  autoware_utils::InterProcessPollingSubscriber<OperationModeState> sub_operational_state_{
    this, "input/operational_mode_state", 1};
  autoware_utils::InterProcessPollingSubscriber<TrafficLightGroupArray> sub_traffic_signals_{
    this, "input/traffic_signals"};

  // publisher
  validator_native::Publisher<Trajectory> pub_traj_;
  validator_native::Publisher<PlanningValidatorStatus> pub_status_;
  validator_native::Publisher<Float64Stamped> pub_processing_time_ms_;
  validator_native::Publisher<visualization_msgs::MarkerArray> pub_markers_;
  validator_native::Publisher<diagnostic_msgs::DiagnosticArray> pub_scene_check_;

  PlanningValidatorManager manager_;

  std::shared_ptr<PlanningValidatorContext> context_;

  bool is_critical_error_ = false;
  bool flag_autonomous_control_enabled_ = false;

  bool isAllValid(const PlanningValidatorStatus & status) const;
  bool infer_autonomous_control_state(const OperationModeState::ConstPtr msg);

  Trajectory::ConstPtr soft_stop_trajectory_;

  std::unique_ptr<autoware_utils::LoggerLevelConfigure> logger_configure_;

  std::unique_ptr<autoware_utils::PublishedTimePublisher> published_time_publisher_;

  StopWatch<std::chrono::milliseconds> stop_watch_;
};
}  // namespace autoware::planning_validator::full_source

#endif  // AUTOWARE__PLANNING_VALIDATOR__NODE_FULL_SOURCE_HPP_
