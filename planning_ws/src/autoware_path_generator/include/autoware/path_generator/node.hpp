#include <autoware/path_generator/ros1_support.hpp>
// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__PATH_GENERATOR__NODE_HPP_
#define AUTOWARE__PATH_GENERATOR__NODE_HPP_

#include <autoware/lanelet2_utils/route_manager.hpp>
#include <autoware/trajectory/path_point_with_lane_id.hpp>
#include <autoware_path_generator/path_generator_parameters.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <autoware_utils_system/stop_watch.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <autoware_internal_debug_msgs/Float64Stamped.h>
#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <autoware_map_msgs/LaneletMapBin.h>
#include <autoware_planning_msgs/LaneletRoute.h>
#include <autoware_vehicle_msgs/HazardLightsCommand.h>
#include <autoware_vehicle_msgs/TurnIndicatorsCommand.h>
#include <nav_msgs/Odometry.h>

#include <memory>
#include <string>

namespace autoware::path_generator
{
using autoware_internal_debug_msgs::Float64Stamped;
using autoware_internal_planning_msgs::PathPointWithLaneId;
using autoware_internal_planning_msgs::PathWithLaneId;
using autoware_map_msgs::LaneletMapBin;
using autoware_planning_msgs::LaneletRoute;
using autoware_vehicle_msgs::HazardLightsCommand;
using autoware_vehicle_msgs::TurnIndicatorsCommand;
using nav_msgs::Odometry;
using ::path_generator::Params;
using Trajectory = autoware::experimental::trajectory::Trajectory<PathPointWithLaneId>;

class PathGenerator
{
public:
  struct RouteManagerData
  {
    LaneletMapBin::ConstPtr lanelet_map_bin_ptr{nullptr};
    LaneletRoute::ConstPtr route_ptr{nullptr};
  };

  explicit PathGenerator(const ros::NodeHandle & node);

  void initialize_route_manager(
    const RouteManagerData & route_manager_data, const geometry_msgs::Pose & initial_pose);

  std::optional<PathWithLaneId> generate_path(
    const geometry_msgs::Pose & current_pose, const Params & params);

private:
  struct InputData
  {
    Odometry::ConstPtr odometry_ptr{nullptr};
  };

  ros::NodeHandle node_;
  ros::Time now() const { return ros::Time::now(); }

  // subscriber
  autoware_utils_rclcpp::InterProcessPollingSubscriber<
    LaneletRoute, autoware_utils_rclcpp::polling_policy::Newest>
    route_subscriber_{&node_, "input/route", 1};
  autoware_utils_rclcpp::InterProcessPollingSubscriber<
    LaneletMapBin, autoware_utils_rclcpp::polling_policy::Newest>
    vector_map_subscriber_{&node_, "input/vector_map", 1};
  autoware_utils_rclcpp::InterProcessPollingSubscriber<Odometry> odometry_subscriber_{
    &node_, "input/odometry"};

  // publisher
  ros::Publisher path_publisher_;
  ros::Publisher turn_signal_publisher_;
  ros::Publisher hazard_signal_publisher_;
  ros::Publisher debug_calculation_time_;

  ros::Timer timer_;

  std::shared_ptr<::path_generator::ParamListener> param_listener_;

  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
  RouteManagerData route_manager_data_;

  std::optional<experimental::lanelet2_utils::RouteManager> route_manager_;

  mutable std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_{nullptr};

  autoware_utils_system::StopWatch<std::chrono::milliseconds> stop_watch_;

  void run();

  InputData take_data();

  bool is_data_ready(const InputData & input_data);

  std::optional<PathWithLaneId> plan_path(
    const geometry_msgs::Pose & current_pose, const Params & params);

  std::optional<PathWithLaneId> generate_path(
    const lanelet::ConstLanelets & lanelet_sequence, const geometry_msgs::Pose & current_pose,
    const double s_ego, const double s_start, const double s_end, const bool connect_to_goal,
    const Params & params) const;

  void publishStopWatchTime();
};
}  // namespace autoware::path_generator

#endif  // AUTOWARE__PATH_GENERATOR__NODE_HPP_
