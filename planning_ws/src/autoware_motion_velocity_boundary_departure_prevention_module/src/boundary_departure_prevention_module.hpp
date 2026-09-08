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

#ifndef BOUNDARY_DEPARTURE_PREVENTION_MODULE_HPP_
#define BOUNDARY_DEPARTURE_PREVENTION_MODULE_HPP_

#include "parameters.hpp"
#include "slow_down_interpolator.hpp"

#include <autoware/motion_velocity_planner_common/plugin_module_interface.hpp>
#include <tl_expected/expected.hpp>

#include <fmt/format.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner::experimental
{
class BoundaryDepartureInputGateOracle;
class BoundaryDeparturePlanningFixtureOracle;

class BoundaryDeparturePreventionModule : public PluginModuleInterface
{
public:
  void init(ros::NodeHandle & node, const std::string & module_name) override;
  void updateParameters(const ParameterMap & parameters) override;
  VelocityPlanningResult plan(
    const TrajectoryPoints & raw_trajectory_points,
    const TrajectoryPoints & smoothed_trajectory_points,
    const std::shared_ptr<const PlannerData> & planner_data) override;
  std::string getModuleName() const override { return module_name_; }
  std::string getShortModuleName() const override { return "boundary_departure_prevention"; }
  RequiredSubscriptionInfo getRequiredSubscriptions() const override
  {
    return RequiredSubscriptionInfo{};
  }
  ~BoundaryDeparturePreventionModule() override { updater_ptr_.reset(); }

private:
  // Candidate-only oracle access. This does not change the plugin's public
  // runtime interface; it lets the transport test assert fail-closed gates
  // after messages traverse real ROS1 subscriptions.
  friend class BoundaryDepartureInputGateOracle;
  friend class BoundaryDeparturePlanningFixtureOracle;

  // === Interface and inputs validation ====
  void subscribe_topics(ros::NodeHandle & node);
  void publish_topics(ros::NodeHandle & node);
  void take_data();
  std::optional<std::string> is_data_invalid(const TrajectoryPoints & raw_trajectory_points) const;
  std::optional<std::string> is_data_timeout(const Odometry & odom) const;
  std::optional<std::string> is_route_changed();
  bool is_autonomous_mode() const;

  // === Internal logic
  /**
   * @brief Main entry point for boundary departure-aware velocity planning.
   *
   * The function first verifies input validity, checks system state (e.g., route changes,
   * autonomy status), and initializes planning components if needed. Then it performs slow down
   * planning to prevent the vehicle from departing road boundaries.
   */

  tl::expected<VelocityPlanningResult, std::string> plan_velocities(
    const TrajectoryPoints & raw_trajectory_points,
    const std::shared_ptr<const PlannerData> & planner_data);

  tl::expected<VelocityPlanningResult, std::string> plan_slow_down_intervals(
    const TrajectoryPoints & raw_trajectory_points,
    const std::shared_ptr<const PlannerData> & planner_data);

  /**
   * @brief Update the list of critical departure points.
   *
   * Projects existing critical departure points onto the updated reference trajectory
   * and removes points that are outdated (i.e., passed by the ego or shifted significantly).
   *
   * @param raw_ref_traj Current reference trajectory.
   * @param offset_from_ego Minimum distance from ego to keep a point; points closer than this are
   * removed.
   */
  std::pair<int8_t, std::string> get_diagnostic_status(
    const double ego_dist_on_traj, const double curr_vel);

  /**
   * @brief Generates and publishes visualization markers for virtual walls and debugging.
   */
  void publish_visualization_markers();

  /**
   * @brief Helper function for virtual walls
   */
  void publish_virtual_walls(const ros::Time & current_time);

  /**
   * @brief Helper function for debug markers
   */
  void publish_debug_markers(const ros::Time & current_time);

  ros::NodeHandle node_;

  std::string module_name_;
  Output output_;
  NodeParam node_param_;
  std::unique_ptr<utils::SlowDownInterpolator> slow_down_interpolator_ptr_;
  MarkerArray debug_marker_;
  MarkerArray slow_down_wall_marker_;
  std::unique_ptr<LaneletRoute> prev_route_ptr_;
  static constexpr auto throttle_duration_ms{5000};

  TrajectoryConstPtr ego_pred_traj_ptr_;
  ControlConstPtr control_cmd_ptr_;
  SteeringReportConstPtr steering_angle_ptr_;
  OperationModeStateConstPtr op_mode_state_ptr_;
  LaneletRouteConstPtr route_ptr_;
  std::unordered_map<std::string, double> processing_times_ms_;

  double last_abnormality_fp_overlap_bound_time_{0.0};
  double last_abnormality_fp_no_overlap_bound_time_{0.0};

  ros::Subscriber ego_pred_traj_sub_;
  ros::Subscriber control_cmd_sub_;
  ros::Subscriber steering_angle_sub_;
  ros::Subscriber op_mode_state_sub_;
  ros::Subscriber route_sub_;
  ros::Publisher debug_trajectory_publisher_;
  ros::Publisher processing_time_detail_pub_;

  std::unique_ptr<UncrossableBoundaryDepartureChecker> boundary_departure_checker_ptr_;
  std::unique_ptr<diagnostic_updater::Updater> updater_ptr_;

  mutable std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_;
};
}  // namespace autoware::motion_velocity_planner::experimental
#endif  // BOUNDARY_DEPARTURE_PREVENTION_MODULE_HPP_
