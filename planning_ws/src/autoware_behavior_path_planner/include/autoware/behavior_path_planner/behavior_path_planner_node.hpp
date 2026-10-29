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

#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER__BEHAVIOR_PATH_PLANNER_NODE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER__BEHAVIOR_PATH_PLANNER_NODE_HPP_

#include <autoware_planning_msgs/StaticObstacleScene.h>

#include "autoware/behavior_path_planner_common/data_manager.hpp"
#include "autoware/behavior_path_planner_common/interface/scene_module_interface.hpp"
#include "autoware_utils/ros/logger_level_configure.hpp"
#include "planner_manager.hpp"

#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils/ros/published_time_publisher.hpp>
#include <autoware_utils_diagnostics/diagnostics_interface.hpp>

#include <autoware_adapi_v1_msgs/OperationModeState.h>
#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <autoware_internal_planning_msgs/Scenario.h>
#include <autoware_internal_planning_msgs/VelocityLimit.h>
#include <autoware_map_msgs/LaneletMapBin.h>
#include <autoware_perception_msgs/PredictedObjects.h>
#include <autoware_planning_msgs/LaneletRoute.h>
#include <autoware_planning_msgs/Path.h>
#include <autoware_planning_msgs/PoseWithUuidStamped.h>
#include <autoware_vehicle_msgs/HazardLightsCommand.h>
#include <autoware_vehicle_msgs/TurnIndicatorsCommand.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tier4_planning_msgs/Approval.h>
#include <tier4_planning_msgs/AvoidanceDebugMsgArray.h>
#include <tier4_planning_msgs/PathChangeModule.h>
#include <tier4_planning_msgs/RerouteAvailability.h>
#include <visualization_msgs/Marker.h>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware::planning_factor_interface::PlanningFactorInterface;
using autoware_adapi_v1_msgs::OperationModeState;
using autoware_internal_planning_msgs::PathWithLaneId;
using autoware_internal_planning_msgs::Scenario;
using autoware_map_msgs::LaneletMapBin;
using autoware_perception_msgs::PredictedObject;
using autoware_perception_msgs::PredictedObjects;
using autoware_perception_msgs::TrafficLightGroupArray;
using autoware_planning_msgs::LaneletRoute;
using autoware_planning_msgs::Path;
using autoware_planning_msgs::PoseWithUuidStamped;
using autoware_vehicle_msgs::HazardLightsCommand;
using autoware_vehicle_msgs::TurnIndicatorsCommand;
using nav_msgs::OccupancyGrid;
using nav_msgs::Odometry;
using geometry_msgs::AccelWithCovarianceStamped;
using autoware_internal_planning_msgs::PlanningFactor;
using autoware_internal_planning_msgs::SafetyFactorArray;
using bpp_full_native::SetParametersResult;
using tier4_planning_msgs::AvoidanceDebugMsgArray;
using tier4_planning_msgs::LateralOffset;
using tier4_planning_msgs::RerouteAvailability;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;
using DebugPublisher = autoware_utils::DebugPublisher;

class BehaviorPathPlannerNode : public bpp_full_native::Node
{
public:
  explicit BehaviorPathPlannerNode(ros::NodeHandle private_node);

  // Getter method for waiting approval modules
  std::vector<std::string> getWaitingApprovalModules();

  // Getter method for running modules
  std::vector<std::string> getRunningModules();

private:
  // subscriber
  autoware_utils::InterProcessPollingSubscriber<
    LaneletRoute, autoware_utils::polling_policy::Newest>
    route_subscriber_{this, "input/route", 1U};
  autoware_utils::InterProcessPollingSubscriber<
    LaneletMapBin, autoware_utils::polling_policy::Newest>
    vector_map_subscriber_{this, "input/vector_map", 1U};
  autoware_utils::InterProcessPollingSubscriber<Odometry> velocity_subscriber_{
    this, "input/odometry"};
  autoware_utils::InterProcessPollingSubscriber<AccelWithCovarianceStamped>
    acceleration_subscriber_{this, "input/accel"};
  autoware_utils::InterProcessPollingSubscriber<Scenario> scenario_subscriber_{
    this, "input/scenario"};
  autoware_utils::InterProcessPollingSubscriber<PredictedObjects> perception_subscriber_{
    this, "input/perception"};
  autoware_utils::InterProcessPollingSubscriber<OccupancyGrid> occupancy_grid_subscriber_{
    this, "input/occupancy_grid_map"};
  autoware_utils::InterProcessPollingSubscriber<OccupancyGrid> costmap_subscriber_{
    this, "input/costmap"};
  autoware_utils::InterProcessPollingSubscriber<TrafficLightGroupArray> traffic_signals_subscriber_{
    this, "input/traffic_signals"};
  autoware_utils::InterProcessPollingSubscriber<LateralOffset> lateral_offset_subscriber_{
    this, "input/lateral_offset"};
  autoware_utils::InterProcessPollingSubscriber<OperationModeState> operation_mode_subscriber_{
    this, "/system/operation_mode/state", 1U};
  autoware_utils::InterProcessPollingSubscriber<autoware_internal_planning_msgs::VelocityLimit>
    external_limit_max_velocity_subscriber_{this, "/planning/scenario_planning/max_velocity"};

  // publisher
  bpp_full_native::Publisher<PathWithLaneId> path_publisher_;
  bpp_full_native::Publisher<PathWithLaneId> longitudinal_commitment_publisher_;
  bpp_full_native::Publisher<std_msgs::Bool> lc_execution_publisher_;
  bpp_full_native::Publisher<std_msgs::Bool> lc_cruise_suppression_publisher_;
  bpp_full_native::Publisher<autoware_planning_msgs::StaticObstacleScene> obstacle_scene_publisher_;
  bpp_full_native::Publisher<TurnIndicatorsCommand> turn_signal_publisher_;
  bpp_full_native::Publisher<HazardLightsCommand> hazard_signal_publisher_;
  bpp_full_native::Publisher<MarkerArray> bound_publisher_;
  bpp_full_native::Publisher<PoseWithUuidStamped> modified_goal_publisher_;
  bpp_full_native::Publisher<RerouteAvailability> reroute_availability_publisher_;
  ros::Timer timer_;

  std::map<std::string, bpp_full_native::Publisher<Path>> path_candidate_publishers_;
  std::map<std::string, bpp_full_native::Publisher<Path>> path_reference_publishers_;

  std::shared_ptr<PlannerData> planner_data_;
  Scenario::ConstPtr current_scenario_{nullptr};
  LaneletMapBin::ConstPtr map_ptr_{nullptr};
  LaneletRoute::ConstPtr route_ptr_{nullptr};
  bool has_received_map_{false};
  bool has_received_route_{false};

  std::shared_ptr<FullSourcePlannerManager> planner_manager_;

  std::unique_ptr<PlanningFactorInterface> planning_factor_interface_;

  std::mutex mutex_pd_;       // mutex for planner_data_
  std::mutex mutex_manager_;  // mutex for bt_manager_ or planner_manager_

  // timeout monitoring
  double cyclic_message_timeout_;  // seconds
  bool enable_traffic_signal_timeout_;
  bool use_occupancy_grid_{true};
  bool enable_turn_signal_output_{true};
  bool enable_hazard_signal_output_{true};
  std::unique_ptr<autoware_utils_diagnostics::DiagnosticsInterface> diagnostics_message_timeout_;
  enum class DataReadyStatus {
    SUCCESS,
    NOT_RECEIVED,
    TIMEOUT,
  };

  // setup
  void takeData();
  DataReadyStatus isDataReady(const ros::Time & now);

  // data processing called from takeData()
  void onTrafficSignals(const TrafficLightGroupArray::ConstPtr msg);
  void onLateralOffset(const LateralOffset::ConstPtr msg);

  SetParametersResult onSetParam(const bpp_full_native::ParameterEvents & parameters);

  ros::Timer parameter_timer_;
  bpp_full_native::ParameterEvents last_parameters_;
  void poll_parameters();

  /**
   * @brief Execute behavior tree and publish planned data.
   */
  void run();

  /**
   * @brief extract path from behavior tree output
   */
  static PathWithLaneId::Ptr getPath(
    const BehaviorModuleOutput & output, const std::shared_ptr<PlannerData> & planner_data);

  /**
   * @brief skip smooth goal connection
   */
  void computeTurnSignal(
    const std::shared_ptr<PlannerData> planner_data, const PathWithLaneId & path,
    const BehaviorModuleOutput & output);

  // debug
  bpp_full_native::Publisher<AvoidanceDebugMsgArray> debug_avoidance_msg_array_publisher_;
  bpp_full_native::Publisher<MarkerArray> debug_turn_signal_info_publisher_;
  std::unique_ptr<DebugPublisher> debug_start_planner_evaluation_table_publisher_ptr_;

  /**
   * @brief publish reroute availability
   */
  void publish_reroute_availability() const;

  /**
   * @brief publish steering factor from intersection
   */
  void publish_steering_factor(
    const std::shared_ptr<PlannerData> & planner_data, const TurnIndicatorsCommand & turn_signal);

  /**
   * @brief publish turn signal debug info
   */
  void publish_turn_signal_debug_data(const TurnSignalDebugData & debug_data);

  /**
   * @brief publish left and right bound
   */
  void publish_bounds(const PathWithLaneId & path);

  /**
   * @brief publish debug messages
   */
  void publishSceneModuleDebugMsg(
    const std::shared_ptr<SceneModuleVisitor> & debug_messages_data_ptr);

  /**
   * @brief publish path candidate
   */
  void publishPathCandidate(
    const std::vector<std::shared_ptr<SceneModuleManagerInterface>> & managers,
    const std::shared_ptr<PlannerData> & planner_data);

  void publishPathReference(
    const std::vector<std::shared_ptr<SceneModuleManagerInterface>> & managers,
    const std::shared_ptr<PlannerData> & planner_data);

  /**
   * @brief convert path with lane id to path for publish path candidate
   */
  Path convertToPath(
    const PathWithLaneId * path_candidate_ptr, const bool is_ready,
    const std::shared_ptr<PlannerData> & planner_data);

  std::unique_ptr<autoware_utils::LoggerLevelConfigure> logger_configure_;

  std::unique_ptr<autoware_utils::PublishedTimePublisher> published_time_publisher_;
};
}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_PLANNER__BEHAVIOR_PATH_PLANNER_NODE_HPP_
