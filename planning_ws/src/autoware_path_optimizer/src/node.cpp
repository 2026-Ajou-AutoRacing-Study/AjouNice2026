// Copyright 2023 TIER IV, Inc.
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

#include "autoware/path_optimizer/obstacle_collision.hpp"

#include "autoware/path_optimizer/node.hpp"
#include "autoware/path_optimizer/horizon_transition.hpp"

#include "autoware/interpolation/spline_interpolation_points_2d.hpp"
#include "autoware/motion_utils/marker/marker_helper.hpp"
#include "autoware/motion_utils/trajectory/conversion.hpp"
#include "autoware/path_optimizer/log_compat.hpp"
#include "autoware/path_optimizer/debug_marker.hpp"
#include "autoware/path_optimizer/utils/geometry_utils.hpp"
#include "autoware/path_optimizer/utils/trajectory_utils.hpp"
#include "autoware/path_optimizer/utils/gaussian_resampling.hpp"
#include "autoware/path_optimizer/ros1_compat.hpp"

#include <boost/crc.hpp>
#include <nlohmann/json.hpp>
#include <ros/serialization.h>
#include <sstream>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <stdexcept>
#include <utility>
#include <tuple>
#include <vector>
#include <tf2/utils.h>
#include <autoware/motion_utils/trajectory/static_obstacle_escape.hpp>
#include <autoware/motion_utils/trajectory/recovery_collision.hpp>

namespace autoware::path_optimizer
{
namespace
{
template <class T>
std::vector<T> concatVectors(const std::vector<T> & prev_vector, const std::vector<T> & next_vector)
{
  std::vector<T> concatenated_vector;
  concatenated_vector.insert(concatenated_vector.end(), prev_vector.begin(), prev_vector.end());
  concatenated_vector.insert(concatenated_vector.end(), next_vector.begin(), next_vector.end());
  return concatenated_vector;
}

[[maybe_unused]] StringStamped createStringStamped(
  const rclcpp::Time & now, const std::string & data)
{
  StringStamped msg;
  msg.stamp = rclcpp::to_builtin_time(now);
  msg.data = data;
  return msg;
}

[[maybe_unused]] Float64Stamped createFloat64Stamped(const rclcpp::Time & now, const float & data)
{
  Float64Stamped msg;
  msg.stamp = rclcpp::to_builtin_time(now);
  msg.data = data;
  return msg;
}

void setZeroVelocityAfterStopPoint(std::vector<TrajectoryPoint> & traj_points)
{
  const auto opt_zero_vel_idx = autoware::motion_utils::searchZeroVelocityIndex(traj_points);
  if (opt_zero_vel_idx) {
    for (size_t i = opt_zero_vel_idx.value(); i < traj_points.size(); ++i) {
      traj_points.at(i).longitudinal_velocity_mps = 0.0;
    }
  }
}

bool hasZeroVelocity(const TrajectoryPoint & traj_point)
{
  constexpr double zero_vel = 0.0001;
  return std::abs(traj_point.longitudinal_velocity_mps) < zero_vel;
}

std::vector<double> calcSegmentLengthVector(const std::vector<TrajectoryPoint> & points)
{
  std::vector<double> segment_length_vector;
  for (size_t i = 0; i < points.size() - 1; ++i) {
    const double segment_length = autoware_utils::calc_distance2d(points.at(i), points.at(i + 1));
    segment_length_vector.push_back(segment_length);
  }
  return segment_length_vector;
}
}  // namespace

PathOptimizer::PathOptimizer(
  const ros::NodeHandle & node_handle, const ros::NodeHandle & private_node)
: Node(node_handle, private_node),
  vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(private_node).getVehicleInfo()),
  debug_data_ptr_(std::make_shared<DebugData>()),
  updater_(node_handle)
{
  // interface publisher
  traj_pub_ = create_publisher<Trajectory>("~/output/path", 1);
  virtual_wall_pub_ = create_publisher<MarkerArray>("~/virtual_wall", 1);

  private_node.param("require_obstacle_scene", require_obstacle_scene_, false);
  private_node.param("obstacle_search_time_budget_ms", obstacle_search_time_budget_ms_, 100.0);
  private_node.param("obstacle_continuity_weight", obstacle_continuity_weight_, 3.0);
  private_node.param("obstacle_candidate_trace_enabled", obstacle_candidate_trace_enabled_, false);
  private_node.param("output_resampling/method", output_resampling_method_, std::string("gaussian"));
  private_node.param("output_resampling/gaussian_sigma_m", output_gaussian_sigma_m_, 0.25);
  if (output_resampling_method_ != "gaussian" && output_resampling_method_ != "akima") {
    throw std::invalid_argument("output_resampling/method must be gaussian or akima");
  }
  if (!std::isfinite(output_gaussian_sigma_m_) || output_gaussian_sigma_m_ <= 0.0) {
    throw std::invalid_argument("output_resampling/gaussian_sigma_m must be finite and positive");
  }
  ROS_INFO("[MPT] output resampling=%s, spatial Gaussian sigma=%.3fm",
    output_resampling_method_.c_str(), output_gaussian_sigma_m_);
  if (!std::isfinite(obstacle_continuity_weight_) || obstacle_continuity_weight_ < 0.0) {
    throw std::invalid_argument("obstacle_continuity_weight must be finite and nonnegative");
  }
  ROS_INFO("[MPT] obstacle_continuity_weight=%.17g", obstacle_continuity_weight_);
  if (!std::isfinite(obstacle_search_time_budget_ms_) || obstacle_search_time_budget_ms_ <= 0.0) {
    throw std::invalid_argument("obstacle_search_time_budget_ms must be finite and positive");
  }
  private_node.param(
    "obstacle_fallback_deceleration", obstacle_fallback_deceleration_, 7.0);
  private_node.param("recovery/enabled", enable_obstacle_fallback_, true);
  RecoveryState::Parameters gate;
  double stopped_speed_kph = 1.0;
  int minimum_cycles = 5;
  private_node.param("recovery/stopped_speed_kph", stopped_speed_kph, 1.0);
  private_node.param("recovery/stopped_duration", gate.stopped_duration, 2.0);
  private_node.param("recovery/failed_duration", gate.failed_duration, 2.0);
  private_node.param("recovery/maximum_sample_gap", gate.maximum_sample_gap, 0.5);
  private_node.param("recovery/minimum_failed_cycles", minimum_cycles, 5);
  private_node.param("recovery/input_max_age", recovery_input_max_age_, 0.5);
  gate.stopped_speed = stopped_speed_kph / 3.6;
  gate.minimum_failed_cycles = minimum_cycles > 0 ? minimum_cycles : 0;
  recovery_state_ = RecoveryState(gate);
  if (!std::isfinite(recovery_input_max_age_) || recovery_input_max_age_ <= 0)
    throw std::invalid_argument("invalid recovery input age");
  ros::NodeHandle recovery_node(node_handle);
  recovery_objects_sub_ = recovery_node.subscribe<autoware_perception_msgs::PredictedObjects>(
    "/perception/object_recognition/objects", 1,
    [this](const autoware_perception_msgs::PredictedObjects::ConstPtr & m) {recovery_objects_ = m;});
  recovery_operation_sub_ = recovery_node.subscribe<autoware_adapi_v1_msgs::OperationModeState>(
    "/system/operation_mode/state", 1,
    [this](const autoware_adapi_v1_msgs::OperationModeState::ConstPtr & m) {
      autonomous_ = m->mode == m->AUTONOMOUS && m->is_autoware_control_enabled && !m->is_in_transition;
      operation_mode_received_ = ros::Time::now();
    });
  control_mode_sub_ = recovery_node.subscribe<std_msgs::Int8>(
    "/vehicle/status/control_mode", 1, [this](const std_msgs::Int8::ConstPtr & m) {
      external_control_ = m->data == 2;  // MORAI AV-ExternalCtrl, same contract as Validator.
      control_mode_received_ = ros::Time::now();
    });
  gear_sub_ = recovery_node.subscribe<std_msgs::Int8>(
    "/vehicle/status/gear", 1, [this](const std_msgs::Int8::ConstPtr & m) {
      drive_gear_ = m->data == 4;  // MORAI D. Parking/preparation is not active driving.
      gear_received_ = ros::Time::now();
    });
  behavior_stop_sub_ = recovery_node.subscribe<std_msgs::Bool>(
    "/planning/recovery/behavior_stop", 1, [this](const std_msgs::Bool::ConstPtr & m) {
      behavior_stop_ = m->data; behavior_stop_received_ = ros::Time::now();
    });
  external_stop_sub_ = recovery_node.subscribe<std_msgs::Bool>(
    "/planning/recovery/external_stop", 1, [this](const std_msgs::Bool::ConstPtr & m) {
      external_stop_ = m->data; external_stop_received_ = ros::Time::now();
    });
  recovery_status_pub_ = recovery_node.advertise<std_msgs::String>("/planning/recovery/status", 10);
  if (!std::isfinite(obstacle_fallback_deceleration_) ||
      obstacle_fallback_deceleration_ <= 0.0) {
    throw std::invalid_argument("obstacle_fallback_deceleration must be finite and positive");
  }
  private_node.param("obstacle_optimization/max_forward_length", obstacle_max_forward_length_, 60.0);
  private_node.param("obstacle_optimization/return_length", obstacle_return_length_, 15.0);
  int clustering_trigger_count = 4;
  private_node.param("obstacle_clustering/trigger_count", clustering_trigger_count, 4);
  private_node.param(
    "obstacle_clustering/polygon_gap_threshold",
    obstacle_clustering_param_.polygon_gap_threshold, 0.75);
  private_node.param(
    "obstacle_clustering/max_longitudinal_span",
    obstacle_clustering_param_.max_longitudinal_span, 4.0);
  private_node.param(
    "obstacle_clustering/max_lateral_span", obstacle_clustering_param_.max_lateral_span, 2.5);
  obstacle_clustering_param_.trigger_count = static_cast<size_t>(std::max(1, clustering_trigger_count));
  if (!std::isfinite(obstacle_max_forward_length_) || obstacle_max_forward_length_ <= 0.0 ||
      !std::isfinite(obstacle_return_length_) || obstacle_return_length_ < 0.0 ||
      !std::isfinite(obstacle_clustering_param_.polygon_gap_threshold) ||
      obstacle_clustering_param_.polygon_gap_threshold < 0.0 ||
      !std::isfinite(obstacle_clustering_param_.max_longitudinal_span) ||
      obstacle_clustering_param_.max_longitudinal_span <= 0.0 ||
      !std::isfinite(obstacle_clustering_param_.max_lateral_span) ||
      obstacle_clustering_param_.max_lateral_span <= 0.0) {
    throw std::invalid_argument("invalid obstacle optimization coverage");
  }
  auto scene_node = private_node;
  obstacle_scene_sub_ = scene_node.subscribe<autoware_planning_msgs::StaticObstacleScene>(
    "/planning/scenario_planning/lane_driving/behavior_planning/static_obstacle_scene", 100,
    [this](const autoware_planning_msgs::StaticObstacleScene::ConstPtr & scene) {
      // The BPP publisher is ordered. On a simulation-clock reset, old future
      // keys would otherwise evict every newly received scene from the cache.
      if (!obstacle_scenes_.empty() && scene->header.stamp.toNSec() < obstacle_scenes_.rbegin()->first) {
        obstacle_scenes_.clear();
        if (pending_path_ && pending_path_->header.stamp > scene->header.stamp) pending_path_.reset();
        resetPreviousData();
      }
      obstacle_scenes_[scene->header.stamp.toNSec()] = *scene;
      while (obstacle_scenes_.size() > 100) obstacle_scenes_.erase(obstacle_scenes_.begin());
      if (pending_path_ && pending_path_->header.stamp == scene->header.stamp) {
        auto path = pending_path_; pending_path_.reset(); onPath(path);
      }
    });
  scene_timeout_timer_ = private_node.createWallTimer(ros::WallDuration(0.05),
    [this](const ros::WallTimerEvent &) {
      if (!pending_path_ || (ros::WallTime::now() - pending_path_since_).toSec() < 0.2) return;
      auto points = trajectory_utils::convertToTrajectoryPoints(pending_path_->points);
      for (auto & point : points) point.longitudinal_velocity_mps = 0.0;
      publishTrajectory(autoware::motion_utils::convertToTrajectory(points, pending_path_->header));
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "[MPT] Missing synchronized static obstacle scene; stopping");
      pending_path_.reset();
    });
  planning_odometry_pub_ = create_publisher<Odometry>("~/debug/planning_odometry", 10);
  // interface subscriber
  path_sub_ = create_subscription<Path>(
    "~/input/path", 1, std::bind(&PathOptimizer::onPath, this, std::placeholders::_1));

  // debug publisher
  publication_evidence_pub_ = create_publisher<StringStamped>("~/debug/publication_evidence", 100);
  obstacle_candidate_trace_pub_ = create_publisher<StringStamped>("~/debug/obstacle_candidate_trace", 100);
  debug_extended_traj_pub_ = create_publisher<Trajectory>("~/debug/extended_traj", 1);
  debug_markers_pub_ = create_publisher<MarkerArray>("~/debug/marker", 1);
  debug_calculation_time_str_pub_ = create_publisher<StringStamped>("~/debug/calculation_time", 1);
  debug_calculation_time_float_pub_ =
    create_publisher<Float64Stamped>("~/debug/processing_time_ms", 1);
  debug_processing_time_detail_pub_ =
    create_publisher<autoware_utils::ProcessingTimeDetail>("~/debug/processing_time_detail_ms", 1);

  {  // parameters
    // parameter for option
    enable_outside_drivable_area_stop_ =
      declare_parameter<bool>("option.enable_outside_drivable_area_stop");
    enable_skip_optimization_ = declare_parameter<bool>("option.enable_skip_optimization");
    enable_reset_prev_optimization_ =
      declare_parameter<bool>("option.enable_reset_prev_optimization");
    use_footprint_polygon_for_outside_drivable_area_check_ =
      declare_parameter<bool>("option.use_footprint_polygon_for_outside_drivable_area_check");
    outside_drivable_area_check_footprint_ =
      declare_parameter<std::string>("option.outside_drivable_area_check_footprint", "rectangle");

    // parameter for debug marker
    enable_pub_debug_marker_ = declare_parameter<bool>("option.debug.enable_pub_debug_marker");
    enable_pub_extra_debug_marker_ =
      declare_parameter<bool>("option.debug.enable_pub_extra_debug_marker");

    // parameter for debug info
    enable_debug_info_ = declare_parameter<bool>("option.debug.enable_debug_info");

    vehicle_stop_margin_outside_drivable_area_ =
      declare_parameter<double>("common.vehicle_stop_margin_outside_drivable_area");

    // parameters for ego nearest search
    ego_nearest_param_ = EgoNearestParam(this);

    // parameters for trajectory
    traj_param_ = TrajectoryParam(this);

    // Diagnostics
    {
      updater_.setHardwareID("path_optimizer");
      updater_.add(
        "path_optimizer_emergency_stop", this, &PathOptimizer::onCheckPathOptimizationValid);
    }
  }

  time_keeper_ =
    std::make_shared<autoware_utils::TimeKeeper>(debug_processing_time_detail_pub_->raw());

  // create core algorithm pointers with parameter declaration
  replan_checker_ptr_ = std::make_shared<ReplanChecker>(this, ego_nearest_param_);
  mpt_optimizer_ptr_ = std::make_shared<MPTOptimizer>(
    this, enable_debug_info_, ego_nearest_param_, vehicle_info_, traj_param_, debug_data_ptr_,
    time_keeper_);

  // reset planners
  // NOTE: This function must be called after core algorithms (e.g. mpt_optimizer_) have been
  // initialized.
  initializePlanning();

  // set parameter callback
  // NOTE: This function must be called after core algorithms (e.g. mpt_optimizer_) have been
  // initialized.
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&PathOptimizer::onParam, this, std::placeholders::_1));

}

rcl_interfaces::msg::SetParametersResult PathOptimizer::onParam(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;

  // parameters for option
  update_param<bool>(
    parameters, "option.enable_outside_drivable_area_stop", enable_outside_drivable_area_stop_);
  update_param<bool>(parameters, "option.enable_skip_optimization", enable_skip_optimization_);
  update_param<bool>(
    parameters, "option.enable_reset_prev_optimization", enable_reset_prev_optimization_);
  update_param<bool>(
    parameters, "option.use_footprint_polygon_for_outside_drivable_area_check",
    use_footprint_polygon_for_outside_drivable_area_check_);
  update_param<std::string>(
    parameters, "option.outside_drivable_area_check_footprint",
    outside_drivable_area_check_footprint_);

  // parameters for debug marker
  update_param<bool>(parameters, "option.debug.enable_pub_debug_marker", enable_pub_debug_marker_);
  update_param<bool>(
    parameters, "option.debug.enable_pub_extra_debug_marker", enable_pub_extra_debug_marker_);

  // parameters for debug info
  update_param<bool>(parameters, "option.debug.enable_debug_info", enable_debug_info_);

  update_param<double>(
    parameters, "common.vehicle_stop_margin_outside_drivable_area",
    vehicle_stop_margin_outside_drivable_area_);

  // parameters for ego nearest search
  ego_nearest_param_.onParam(parameters);

  // parameters for trajectory
  traj_param_.onParam(parameters);

  // parameters for core algorithms
  replan_checker_ptr_->onParam(parameters);
  mpt_optimizer_ptr_->onParam(parameters);

  // reset planners
  initializePlanning();

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void PathOptimizer::initializePlanning()
{
  RCLCPP_DEBUG(get_logger(), "Initialize planning");

  mpt_optimizer_ptr_->initialize(enable_debug_info_, traj_param_);

  resetPreviousData();
}

void PathOptimizer::resetPreviousData()
{
  mpt_optimizer_ptr_->resetPreviousData();
  last_successful_optimization_time_.reset();
  last_validated_output_.reset();
  using_final_geometry_ = false;
  obstacle_search_cursor_ = 0;
  relaxed_obstacle_search_cursor_ = 0;
  retained_optimization_forward_length_ = 0.0;
  // Recovery evidence and latch belong to observed planning results, not seed history.
}

void PathOptimizer::onPath(const PathConstPtr path_ptr)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  stop_watch_.tic();

  // check if input path is valid
  if (!checkInputPath(*path_ptr, *get_clock())) {
    return;
  }

  if (require_obstacle_scene_ && !obstacle_scenes_.count(path_ptr->header.stamp.toNSec())) {
    if (!pending_path_) pending_path_since_ = ros::WallTime::now();
    pending_path_ = path_ptr;
    return;
  }
  pending_path_.reset();
  if (require_obstacle_scene_ &&
      obstacle_scenes_.at(path_ptr->header.stamp.toNSec()).header.frame_id != path_ptr->header.frame_id) {
    auto points = trajectory_utils::convertToTrajectoryPoints(path_ptr->points);
    for (auto & point : points) point.longitudinal_velocity_mps = 0.0;
    publishTrajectory(autoware::motion_utils::convertToTrajectory(points, path_ptr->header));
    RCLCPP_ERROR(get_logger(), "[MPT] Obstacle scene frame mismatch; stopping");
    return;
  }

  // check if ego's odometry is valid
  const auto ego_odom_ptr = ego_odom_sub_.take_data();
  if (!ego_odom_ptr) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), 5000, "Waiting for ego pose and twist.");
    return;
  }

  // 0. return if path is backward
  // TODO(murooka): support backward path
  const auto is_driving_forward = driving_direction_checker_.isDrivingForward(path_ptr->points);
  if (!is_driving_forward) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Backward path is NOT supported. Just converting path to trajectory");

    auto traj_points = trajectory_utils::convertToTrajectoryPoints(path_ptr->points);
    if (require_obstacle_scene_) {
      // This optimizer cannot establish reverse obstacle avoidance feasibility.
      for (auto & point : traj_points) point.longitudinal_velocity_mps = 0.0;
    }
    const auto output_traj_msg =
      autoware::motion_utils::convertToTrajectory(traj_points, path_ptr->header);
    publishTrajectory(output_traj_msg);
    return;
  }

  // 1. create planner data
  const auto planner_data = createPlannerData(*path_ptr, ego_odom_ptr);
  const auto fresh = [&](const ros::Time & stamp) {
    const double age = (ros::Time::now() - stamp).toSec();
    return !stamp.isZero() && age >= -0.05 && age <= recovery_input_max_age_;
  };
  const size_t recovery_ego = trajectory_utils::findEgoIndex(
    planner_data.traj_points, planner_data.ego_pose, ego_nearest_param_);
  const auto recovery_scene = obstacle_scenes_.find(path_ptr->header.stamp.toNSec());
  recovery_permitted_ = enable_obstacle_fallback_ &&
    recovery_scene != obstacle_scenes_.end() &&
    recovery_scene->second.header.frame_id == path_ptr->header.frame_id &&
    planner_data.traj_points[recovery_ego].longitudinal_velocity_mps > 0.1 &&
    autonomous_ && fresh(operation_mode_received_) && external_control_ && fresh(control_mode_received_) &&
    drive_gear_ && fresh(gear_received_) &&
    fresh(external_stop_received_) && !external_stop_ &&
    fresh(behavior_stop_received_) && !behavior_stop_ &&
    recovery_objects_ && recovery_objects_->header.frame_id == path_ptr->header.frame_id &&
    fresh(recovery_objects_->header.stamp) && fresh(ego_odom_ptr->header.stamp) &&
    fresh(path_ptr->header.stamp);
  recovery_cycle_time_ = now().toSec();
  recovery_state_.observe(recovery_cycle_time_, planner_data.ego_vel,
    fresh(ego_odom_ptr->header.stamp) && fresh(path_ptr->header.stamp), recovery_permitted_);
  auto planning_odometry = *ego_odom_ptr;
  planning_odometry.header = path_ptr->header;  // Key debug input to the output trajectory.
  planning_odometry_pub_->publish(planning_odometry);

  // 2. generate optimized trajectory
  const auto optimized_traj_points = generateOptimizedTrajectory(planner_data);

  // 3. extend trajectory to connect the optimized trajectory and the following path smoothly
  auto full_traj_points = using_final_geometry_ ? optimized_traj_points :
    extendTrajectory(planner_data.traj_points, optimized_traj_points);

  // Road and obstacle checks must see the same geometry that is published.
  // Extension reconstructs yaw; checking the raw MPT states can insert a stop
  // even though the final vehicle footprint is inside the road (or vice versa).
  if (!using_recovery_geometry_) insertZeroVelocityOutsideDrivableArea(planner_data, full_traj_points);

  // Validate after extension/resampling too: the optimized prefix alone is insufficient.
  if (stopBeforeObstacle(planner_data, full_traj_points)) is_optimization_failed_ = true;
  if (using_recovery_geometry_ &&
      (!recovery_permitted_ || !validateRecovery(planner_data, full_traj_points))) {
    for (auto & point : full_traj_points) point.longitudinal_velocity_mps = 0.0;
    using_recovery_geometry_ = false;
    is_optimization_failed_ = true;
    selected_stop_ = true;
  }

  // 4. set zero velocity after stop point
  setZeroVelocityAfterStopPoint(full_traj_points);
  recovery_state_.normalResult(recovery_cycle_time_,
    normal_fresh_solution_ && !is_optimization_failed_ && validateNormal(planner_data, full_traj_points),
    selected_stop_, recovery_permitted_);
  publishRecoveryStatus(planner_data);
  if (!is_optimization_failed_) {
    last_validated_output_ = full_traj_points;
    last_successful_optimization_time_ = now();
  }

  // 5. publish debug data
  publishDebugMarkerOfOptimization(full_traj_points);
  publishDebugData(planner_data.header);
  updater_.force_update();

  // publish calculation_time
  // NOTE: This function must be called after measuring onPath calculation time
  debug_calculation_time_float_pub_->publish(createFloat64Stamped(now(), stop_watch_.toc()));

  const auto output_traj_msg =
    autoware::motion_utils::convertToTrajectory(full_traj_points, path_ptr->header);
  auto recovery_output = output_traj_msg;
  recovery_output.boundary_recovery = using_recovery_geometry_ && recovery_state_.active();
  publishTrajectory(recovery_output);
  if (obstacle_candidate_trace_enabled_ && !obstacle_candidate_trace_.empty()) {
    obstacle_candidate_trace_pub_->publish(
      createStringStamped(path_ptr->header.stamp, obstacle_candidate_trace_));
  }
}

void PathOptimizer::publishTrajectory(Trajectory trajectory)
{
  // A Python recorder can stall across multiple topic callbacks. Measure the
  // actual ROS publish call and bind its timing to the exact serialized output.
  trajectory.header.seq = 0;
  const auto serialized = ros::serialization::serializeMessage(trajectory);
  boost::crc_32_type checksum;
  checksum.process_bytes(serialized.message_start, serialized.num_bytes - 4);
  const auto wall_ns = [] {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  };
  const auto before = wall_ns();
  traj_pub_->publish(trajectory);
  const auto after = wall_ns();
  std::ostringstream values;
  values << publication_sequence_++ << ' ' << checksum.checksum() << ' '
         << before << ' ' << after;
  StringStamped evidence;
  evidence.stamp = rclcpp::to_builtin_time(trajectory.header.stamp);
  evidence.data = values.str();
  publication_evidence_pub_->publish(evidence);
}

bool PathOptimizer::validateNormal(
  const PlannerData & data, const std::vector<TrajectoryPoint> & points) const
{
  if (points.size()<3) return false;
  const size_t ego=trajectory_utils::findEgoIndex(points,data.ego_pose,ego_nearest_param_);
  if (!isAlignedWithEgoPose(points[ego].pose,data.ego_pose)) return false;
  FullFootprintRoadChecker road(data.left_bound,data.right_bound);
  if (!road.contains(data.ego_pose,vehicle_info_)) return false;
  const size_t count=static_cast<size_t>(mpt_optimizer_ptr_->getNumberOfPoints()*
    mpt_optimizer_ptr_->getDeltaArcLength()/traj_param_.output_delta_arc_length);
  const size_t end=std::min(points.size()>5?points.size()-5:points.size(),ego+count);
  if(end<=ego+1)return false;
  for(size_t i=ego;i<end;++i) if(!road.contains(points[i].pose,vehicle_info_))return false;
  if(firstObstacleCollision(points,ego,data.obstacles,vehicle_info_) ||
     firstCurvatureViolation(points,ego,end,mpt_optimizer_ptr_->maximumCurvature()))return false;
  return !recovery_state_.active() || validateRecovery(data,points);
}

bool PathOptimizer::validateRecovery(
  const PlannerData & data, const std::vector<TrajectoryPoint> & points) const
{
  if(points.size()<3)return false;
  const size_t ego=trajectory_utils::findEgoIndex(points,data.ego_pose,ego_nearest_param_);
  if(!isAlignedWithEgoPose(points[ego].pose,data.ego_pose) ||
     firstObstacleCollision(points,ego,data.obstacles,vehicle_info_) ||
     firstCurvatureViolation(points,ego,points.size(),mpt_optimizer_ptr_->maximumCurvature()))return false;
  if(!recovery_objects_)return false;
  const double age=(ros::Time::now()-recovery_objects_->header.stamp).toSec();
  if(age < -0.05 || age > recovery_input_max_age_ ||
      recovery_objects_->header.frame_id!=data.header.frame_id)return false;
  return autoware::motion_utils::recoveryCollisionFree(
    points,data.ego_pose,data.ego_vel,*recovery_objects_,vehicle_info_,std::max(0.0,age));
}

void PathOptimizer::publishRecoveryStatus(const PlannerData & data)
{
  std_msgs::String status;
  status.data=nlohmann::json{{"stamp",data.header.stamp.toSec()},
    {"active",recovery_state_.active()}, {"permitted",recovery_permitted_},
    {"using_recovery_trajectory",using_recovery_geometry_},
    {"normal_fresh_success",normal_fresh_solution_ && !is_optimization_failed_},
    {"previous_reused",selected_previous_}, {"optimizer_stop",selected_stop_},
    {"normal_optimization_failed",normal_optimization_failed_},
    {"autonomous",autonomous_}, {"external_control",external_control_}, {"drive_gear",drive_gear_},
    {"behavior_stop",behavior_stop_}, {"object_stop",external_stop_},
    {"failed_cycles",recovery_state_.failedCycles()},
    {"stopped_seconds",recovery_state_.stoppedSeconds(recovery_cycle_time_)},
    {"failed_seconds",recovery_state_.failedSeconds(recovery_cycle_time_)}}.dump();
  recovery_status_pub_.publish(status);
}

bool PathOptimizer::checkInputPath(const Path & path, rclcpp::Clock clock) const
{
  if (path.points.size() < 2) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), clock, 5000, "Path points size is less than 1.");
    return false;
  }

  if (path.left_bound.empty() || path.right_bound.empty()) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(
      get_logger(), clock, 5000, "Left or right bound in path is empty.");
    return false;
  }

  return true;
}

void PathOptimizer::onCheckPathOptimizationValid(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (using_recovery_geometry_) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,
      "[Path Optimizer]: road-bound fallback trajectory in use");
    return;
  }
  if (is_optimization_failed_) {
    const std::string error_msg =
      "[Path Optimizer]: MPT failed; using checked previous path or stopping";
    const auto diag_level = diagnostic_msgs::DiagnosticStatus::ERROR;
    stat.summary(diag_level, error_msg);
  } else {
    const std::string error_msg = "[Path Optimizer]: MPT Optimizer successful";
    const auto diag_level = diagnostic_msgs::DiagnosticStatus::OK;
    stat.summary(diag_level, error_msg);
  }
}

PlannerData PathOptimizer::createPlannerData(
  const Path & path, const OdometryConstPtr ego_odom_ptr) const
{
  // create planner data
  PlannerData planner_data;
  planner_data.header = path.header;
  planner_data.traj_points = trajectory_utils::convertToTrajectoryPoints(path.points);
  planner_data.left_bound = path.left_bound;
  planner_data.right_bound = path.right_bound;
  planner_data.allow_initial_obstacle_overlap = false;
  const auto scene = obstacle_scenes_.find(path.header.stamp.toNSec());
  if (scene != obstacle_scenes_.end() && scene->second.header.frame_id == path.header.frame_id) {
    planner_data.obstacles = scene->second.obstacles;
  }
  planner_data.ego_pose = ego_odom_ptr->pose.pose;
  planner_data.ego_vel = ego_odom_ptr->twist.twist.linear.x;

  if (!planner_data.obstacles.empty()) {
    // A short QP followed by an unoptimized centreline cannot avoid an obstacle
    // in that suffix. Cover the observed object AND the return before joining.
    const auto & points = planner_data.traj_points;
    std::vector<double> arc(points.size(), 0.0);
    for (size_t i = 1; i < points.size(); ++i) {
      arc[i] = arc[i - 1] + autoware_utils::calc_distance2d(points[i - 1], points[i]);
    }
    const size_t ego = trajectory_utils::findEgoIndex(points, planner_data.ego_pose, ego_nearest_param_);
    double required = mpt_optimizer_ptr_->getNumberOfPoints() * mpt_optimizer_ptr_->getDeltaArcLength();
    for (const auto & obstacle : planner_data.obstacles) {
      for (const auto & vertex : obstacle.polygon.points) {
        geometry_msgs::Point point;
        point.x = vertex.x; point.y = vertex.y;
        const auto index = autoware::motion_utils::findNearestIndex(points, point);
        const double ahead = arc[index] - arc[ego];
        if (ahead >= 0.0) required = std::max(required, ahead + obstacle.hard_margin +
          obstacle.soft_margin + vehicle_info_.vehicle_length_m + obstacle_return_length_);
      }
    }
    planner_data.optimization_forward_length = std::min(required, obstacle_max_forward_length_);
  }

  debug_data_ptr_->ego_pose = planner_data.ego_pose;
  return planner_data;
}

std::vector<TrajectoryPoint> PathOptimizer::generateOptimizedTrajectory(
  const PlannerData & planner_data)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // 1. calculate trajectory with MPT
  //    NOTE: This function may return previously optimized trajectory points.
  //          Also, velocity on some points will not be updated for a logic purpose.
  auto optimized_traj_points = optimizeTrajectory(planner_data);

  return optimized_traj_points;
}

std::vector<TrajectoryPoint> PathOptimizer::optimizeTrajectory(const PlannerData & planner_data)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto & p = planner_data;
  using_final_geometry_ = false;
  using_recovery_geometry_ = false;
  normal_fresh_solution_ = false;
  normal_optimization_failed_ = false;
  selected_previous_ = false;
  selected_stop_ = false;
  normal_obstacle_search_complete_ = false;
  obstacle_candidate_trace_.clear();
  optimization_reset_reason_.clear();

  TrajectoryPoint measured;
  measured.pose = p.ego_pose;
  if (!p.allow_initial_obstacle_overlap &&
      firstObstacleCollision({measured}, 0, p.obstacles, vehicle_info_)) {
    is_optimization_failed_ = true;
    auto stopped = p.traj_points;
    for (auto & point : stopped) point.longitudinal_velocity_mps = 0.0;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "[MPT] Measured footprint already overlaps static obstacle clearance; stopping");
    return stopped;
  }

  // 1. check if replan (= optimization) is required
  const bool is_replan_required = [&]() {
    const bool reset_prev_optimization = replan_checker_ptr_->isResetRequired(planner_data);
    if (enable_reset_prev_optimization_ || reset_prev_optimization) {
      optimization_reset_reason_ = enable_reset_prev_optimization_
        ? "forced_reset_parameter" : replan_checker_ptr_->lastResetReason();
      // NOTE: always replan when resetting previous optimization
      resetPreviousData();
      return true;
    }
    // check replan when not resetting previous optimization
    // Retry after fallback; do not resurrect an older optimized path on the next cycle.
    return recovery_state_.active() || !p.obstacles.empty() || is_optimization_failed_ || replan_checker_ptr_->isReplanRequired(planner_data, now());
  }();
  replan_checker_ptr_->updateData(planner_data, is_replan_required, now());
  if (!is_replan_required) {
    // Reuse spatial optimization only. BPP/BVP may update an LC deceleration,
    // stop, or released limit without changing the reference geometry.
    auto reused = getPrevOptimizedTrajectory(p.traj_points);
    applyInputVelocity(reused, p.traj_points, p.ego_pose);
    return reused;
  }

  if (enable_skip_optimization_ && p.obstacles.empty() && !recovery_state_.active()) {
    is_optimization_failed_ = false;
    return p.traj_points;
  }

  // 2. make trajectory kinematically-feasible and collision-free (= inside the drivable area)
  //    with model predictive trajectory
  if (recovery_state_.active()) mpt_optimizer_ptr_->restoreHistory(MPTOptimizer::History{});
  const auto history_before_optimization = mpt_optimizer_ptr_->history();
  const auto optimization_started = std::chrono::steady_clock::now();
  // Normal optimization never consumes a boundary-relaxed candidate. Recovery
  // is a persistent authorization, while output provenance is per trajectory.
  const double normal_budget = obstacle_search_time_budget_ms_ *
    (recovery_state_.active() ? 0.5 : 1.0);
  auto mpt_traj = p.obstacles.empty()
    ? mpt_optimizer_ptr_->optimizeTrajectory(planner_data)
    : optimizeObstacleCandidates(planner_data, normal_budget);
  const double base_length =
    mpt_optimizer_ptr_->getNumberOfPoints() * mpt_optimizer_ptr_->getDeltaArcLength();
  if (!p.obstacles.empty()) retained_optimization_forward_length_ = 0.0;
  if (p.obstacles.empty() && mpt_traj &&
      retained_optimization_forward_length_ > base_length && last_validated_output_) {
    auto prefix = *mpt_traj;
    applyInputVelocity(prefix, p.traj_points, p.ego_pose);
    const auto candidate = extendTrajectory(p.traj_points, prefix);
    if (isHorizonReturnContinuous(
          *last_validated_output_, candidate, p.ego_pose, std::max(10.0, base_length))) {
      retained_optimization_forward_length_ = 0.0;
    } else {
      // A shorter feasible QP can move the PP target abruptly. Keep solving the
      // longer window until its published near field agrees with the short one.
      mpt_traj.reset();
      mpt_optimizer_ptr_->restoreHistory(history_before_optimization);
    }
  }
  // A terminal match can be infeasible in the short nominal window. Solve a
  // longer window instead of bridging back to the input outside the QP.
  if (!mpt_traj && p.obstacles.empty()) {
    for (double forward = std::max(base_length + 10.0, retained_optimization_forward_length_);
         forward <= obstacle_max_forward_length_ + 1e-6; forward += 10.0) {
      const double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - optimization_started).count();
      if (elapsed_ms >= normal_budget) break;
      auto longer = planner_data;
      longer.optimization_forward_length = forward;
      mpt_optimizer_ptr_->restoreHistory(history_before_optimization);
      mpt_traj = mpt_optimizer_ptr_->optimizeTrajectory(longer);
      if (mpt_traj) {
        retained_optimization_forward_length_ = forward;
        break;
      }
    }
  }

  if (mpt_traj) {
    auto checked = *mpt_traj;
    if (!using_final_geometry_) {
      applyInputVelocity(checked, p.traj_points, p.ego_pose);
      checked = extendTrajectory(p.traj_points, checked);
    }
    if (recovery_state_.active() && !validateNormal(p, checked)) mpt_traj.reset();
    else {mpt_traj = std::move(checked); using_final_geometry_ = true;}
  }
  normal_fresh_solution_ = mpt_traj.has_value() && !selected_previous_;
  normal_optimization_failed_ = !mpt_traj;
  const auto normal_history = mpt_traj ? mpt_optimizer_ptr_->history() : history_before_optimization;
  if (!mpt_traj && recovery_state_.active() && recovery_permitted_) {
    const double spent = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - optimization_started).count();
    const double budget = std::max(0.0, obstacle_search_time_budget_ms_ - spent);
    { // Guarantee one relaxed solve even when the normal solver overruns its budget.
      // Candidate search already limits subsequent attempts against this budget.
      auto relaxed = p;
      relaxed.relax_road_bounds = true;
      relaxed.allow_initial_obstacle_overlap = false;
      mpt_optimizer_ptr_->restoreHistory(MPTOptimizer::History{});
      using_final_geometry_ = false;
      mpt_traj = p.obstacles.empty()
        ? mpt_optimizer_ptr_->optimizeTrajectory(relaxed)
        : optimizeObstacleCandidates(relaxed, budget);
      if (mpt_traj) {
        auto checked = *mpt_traj;
        if (!using_final_geometry_) {
          applyInputVelocity(checked, p.traj_points, p.ego_pose);
          checked = extendTrajectory(p.traj_points, checked);
        }
        if (!validateRecovery(p, checked)) mpt_traj.reset();
        else {
          mpt_traj = std::move(checked);
          using_final_geometry_ = true;
          using_recovery_geometry_ = true;
        }
      }
    }
  }
  // A relaxed solve must never become the next normal solve's fixed prefix.
  mpt_optimizer_ptr_->restoreHistory(normal_history);
  is_optimization_failed_ = !static_cast<bool>(mpt_traj);
  if (mpt_traj) {
    auto optimized_traj_points = *mpt_traj;
    if (!using_final_geometry_) {
      applyInputVelocity(optimized_traj_points, p.traj_points, planner_data.ego_pose);
    }
    return optimized_traj_points;
  }

  // A failed QP must not switch from an ongoing avoidance path to the unoptimized centerline.
  // Recheck the last optimized path against this cycle's corridor and ego pose before using it.
  // Only the published, postprocessed geometry was collision/curvature checked.
  // Raw MPT states have different headings and must not be reused as that path.
  if (const auto & previous = last_validated_output_; previous && previous->size() >= 2) {
    auto fallback = *previous;
    applyInputVelocity(fallback, p.traj_points, p.ego_pose);
    if (fallback.size() >= 2) {
      const size_t ego_idx = trajectory_utils::findEgoIndex(fallback, p.ego_pose, ego_nearest_param_);
      const auto & nearest_pose = fallback.at(ego_idx).pose;
      if (isAlignedWithEgoPose(nearest_pose, p.ego_pose)) {
        const auto radii = mpt_optimizer_ptr_->getVehicleCircleRadii();
        const auto offsets = mpt_optimizer_ptr_->getVehicleCircleLongitudinalOffsets();
        const size_t check_end = fallback.size() > 5 ? fallback.size() - 5 : fallback.size();
        const geometry_utils::CircleFootprintRoadChecker road(p.left_bound, p.right_bound);
        std::optional<size_t> first_outside_idx;
        for (size_t i = ego_idx; i < check_end; ++i) {
          if (road.isOutside(fallback.at(i).pose, radii, offsets)) {
            first_outside_idx = i;
            break;
          }
        }
        if (const auto collision = firstObstacleCollision(fallback, ego_idx, p.obstacles, vehicle_info_)) {
          const bool escaping = p.allow_initial_obstacle_overlap &&
            autoware::motion_utils::checkStaticObstacleEscape(
              fallback, p.ego_pose, p.obstacles, vehicle_info_).valid;
          if (!escaping && (!first_outside_idx || *collision < *first_outside_idx)) first_outside_idx = collision;
        }
        const bool inside_current_corridor = !first_outside_idx;
        const double age = last_successful_optimization_time_
          ? (now() - *last_successful_optimization_time_).toSec()
          : std::numeric_limits<double>::infinity();
        // Fresh scene/road checks, not the age of the original QP, decide whether
        // this spatial curve is still usable. An arbitrary age timeout must not
        // stop an otherwise revalidated avoidance maneuver. Retry the QP each cycle.
        if (inside_current_corridor && std::isfinite(age) && age >= 0.0) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "[MPT] Optimization failed; holding validated previous path");
          using_final_geometry_ = true;
          return fallback;
        }
        // A stale or newly blocked path can still supply a continuous stop prefix. Reserve
        // one metre before the first invalid point and use the configured braking envelope.
        const double braking_deceleration = obstacle_fallback_deceleration_;
        constexpr double stop_margin = 1.0;
        const size_t last_safe_idx = first_outside_idx ? *first_outside_idx : check_end - 1;
        double safe_distance = 0.0;
        for (size_t i = ego_idx + 1; i <= last_safe_idx; ++i) {
          safe_distance += autoware_utils::calc_distance2d(
            fallback.at(i - 1).pose, fallback.at(i).pose);
        }
        const double stop_distance = std::max(0.0, safe_distance - stop_margin);
        const double required_distance = p.ego_vel * p.ego_vel / (2.0 * braking_deceleration);
        if (stop_distance >= required_distance) {
          double distance = 0.0;
          for (size_t i = ego_idx; i < fallback.size(); ++i) {
            if (i > ego_idx) {
              distance += autoware_utils::calc_distance2d(
                fallback.at(i - 1).pose, fallback.at(i).pose);
            }
            const double allowed_velocity = std::sqrt(
              2.0 * braking_deceleration * std::max(0.0, stop_distance - distance));
            fallback.at(i).longitudinal_velocity_mps = std::min(
              static_cast<double>(fallback.at(i).longitudinal_velocity_mps), allowed_velocity);
          }
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[MPT] Optimization failed; braking on previous path (clear=%.2f stop=%.2f)",
            safe_distance, stop_distance);
        } else {
          for (auto & point : fallback) {
            point.longitudinal_velocity_mps = 0.0;
          }
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[MPT] Optimization failed; emergency brake, previous path clear distance %.2f < %.2f",
            safe_distance, required_distance);
        }
        using_final_geometry_ = true;
        selected_stop_ = true;
        return fallback;
      }
    }
  }
  selected_stop_ = true;
  auto stop_on_input = p.traj_points;
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "[MPT] Optimization failed; no aligned previous path, emergency stop on input path");
  applyInputVelocity(stop_on_input, p.traj_points, p.ego_pose);
  for (auto & point : stop_on_input) {
    point.longitudinal_velocity_mps = 0.0;
  }
  return stop_on_input;
}

std::vector<TrajectoryPoint> PathOptimizer::getPrevOptimizedTrajectory(
  const std::vector<TrajectoryPoint> & traj_points) const
{
  const auto prev_optimized_traj_points = mpt_optimizer_ptr_->getPrevOptimizedTrajectoryPoints();
  if (prev_optimized_traj_points) {
    return *prev_optimized_traj_points;
  }
  return traj_points;
}

void PathOptimizer::applyInputVelocity(
  std::vector<TrajectoryPoint> & output_traj_points,
  const std::vector<TrajectoryPoint> & input_traj_points,
  const geometry_msgs::Pose & ego_pose) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // trim to ego-pose
  const size_t ego_seg_idx_output_traj =
    trajectory_utils::findEgoSegmentIndex(output_traj_points, ego_pose, ego_nearest_param_);
  output_traj_points = autoware::motion_utils::cropBackwardPoints(
    output_traj_points, ego_pose.position, ego_seg_idx_output_traj,
    traj_param_.output_backward_traj_length);

  // crop forward for faster calculation
  const auto forward_cropped_input_traj_points = [&]() {
    double optimized_traj_length = mpt_optimizer_ptr_->getTrajectoryLength();
    double actual_length = 0.0;
    for (size_t i = 1; i < output_traj_points.size(); ++i) {
      actual_length += autoware_utils::calc_distance2d(output_traj_points[i - 1], output_traj_points[i]);
    }
    optimized_traj_length = std::max(optimized_traj_length, actual_length);
    constexpr double margin_traj_length = 10.0;

    const size_t ego_seg_idx =
      trajectory_utils::findEgoSegmentIndex(input_traj_points, ego_pose, ego_nearest_param_);
    const auto cropped_points = autoware::motion_utils::cropForwardPoints(
      input_traj_points, ego_pose.position, ego_seg_idx,
      optimized_traj_length + margin_traj_length);

    if (cropped_points.size() < 2) {
      return input_traj_points;
    }
    return cropped_points;
  }();

  // update velocity
  const auto segment_length_vec = calcSegmentLengthVector(forward_cropped_input_traj_points);
  const double mpt_delta_arc_length = mpt_optimizer_ptr_->getDeltaArcLength();
  size_t input_traj_start_idx = trajectory_utils::findEgoSegmentIndex(
    forward_cropped_input_traj_points, output_traj_points.front().pose, ego_nearest_param_);
  for (size_t i = 0; i < output_traj_points.size(); i++) {
    // NOTE: input_traj_start/end_idx is calculated for efficient index calculation
    const size_t input_traj_end_idx = [&]() {
      double sum_segment_length = 0.0;
      for (size_t j = input_traj_start_idx + 1; j < segment_length_vec.size(); ++j) {
        sum_segment_length += segment_length_vec.at(j);
        if (mpt_delta_arc_length < sum_segment_length) {
          return j + 1;
        }
      }
      return forward_cropped_input_traj_points.size() - 1;
    }();

    const auto nearest_traj_point = [&]() {
      if (input_traj_start_idx == input_traj_end_idx) {
        return forward_cropped_input_traj_points.at(input_traj_start_idx);
      }

      // crop forward and backward for efficient calculation
      const auto cropped_input_traj_points = std::vector<TrajectoryPoint>{
        forward_cropped_input_traj_points.begin() + input_traj_start_idx,
        forward_cropped_input_traj_points.begin() + input_traj_end_idx + 1};
      assert(2 <= cropped_input_traj_points.size());

      const size_t nearest_seg_idx = trajectory_utils::findEgoSegmentIndex(
        cropped_input_traj_points, output_traj_points.at(i).pose, ego_nearest_param_);
      input_traj_start_idx += nearest_seg_idx;

      return cropped_input_traj_points.at(nearest_seg_idx);
    }();

    // calculate velocity with zero order hold
    output_traj_points.at(i).longitudinal_velocity_mps =
      nearest_traj_point.longitudinal_velocity_mps;
  }

  // insert stop point explicitly
  const auto stop_idx =
    autoware::motion_utils::searchZeroVelocityIndex(forward_cropped_input_traj_points);
  if (stop_idx) {
    const auto & input_stop_pose = forward_cropped_input_traj_points.at(stop_idx.value()).pose;
    // NOTE: autoware::motion_utils::findNearestSegmentIndex is used instead of
    // trajectory_utils::findEgoSegmentIndex
    //       for the case where input_traj_points is much longer than output_traj_points, and the
    //       former has a stop point but the latter will not have.
    const auto stop_seg_idx = autoware::motion_utils::findNearestSegmentIndex(
      output_traj_points, input_stop_pose, ego_nearest_param_.dist_threshold,
      ego_nearest_param_.yaw_threshold);

    // calculate and insert stop pose on output trajectory
    const bool is_stop_point_inside_trajectory = [&]() {
      if (!stop_seg_idx) {
        return false;
      }
      if (*stop_seg_idx == output_traj_points.size() - 2) {
        const double signed_projected_length_to_segment =
          autoware::motion_utils::calcLongitudinalOffsetToSegment(
            output_traj_points, *stop_seg_idx, input_stop_pose.position);
        const double segment_length = autoware::motion_utils::calcSignedArcLength(
          output_traj_points, *stop_seg_idx, *stop_seg_idx + 1);
        if (segment_length < signed_projected_length_to_segment) {
          // NOTE: input_stop_pose is outside output_traj_points.
          return false;
        }
      }
      return true;
    }();
    if (is_stop_point_inside_trajectory) {
      trajectory_utils::insertStopPoint(output_traj_points, input_stop_pose, *stop_seg_idx);
    }
  }
}

void PathOptimizer::insertZeroVelocityOutsideDrivableArea(
  const PlannerData & planner_data, std::vector<TrajectoryPoint> & optimized_traj_points) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  if (optimized_traj_points.empty()) {
    return;
  }

  // 1. calculate ego_index nearest to optimized_traj_points
  const size_t ego_idx = trajectory_utils::findEgoIndex(
    optimized_traj_points, planner_data.ego_pose, ego_nearest_param_);

  // 2. calculate an end point to check being outside the drivable area
  // NOTE: Some terminal trajectory points tend to be outside the drivable area when
  //       they have high curvature.
  //       Therefore, these points should be ignored to check if being outside the drivable area
  constexpr int num_points_ignore_drivable_area = 5;
  // The final path has a different sample interval and can include a long
  // unoptimized tail. Preserve the optimized horizon in metres, not indices.
  const double check_distance = std::max(0.0,
    (mpt_optimizer_ptr_->getNumberOfPoints() - num_points_ignore_drivable_area) *
    mpt_optimizer_ptr_->getDeltaArcLength());
  size_t end_idx = ego_idx;
  double distance = 0.0;
  while (end_idx + 1 < optimized_traj_points.size() && distance < check_distance) {
    distance += autoware_utils::calc_distance2d(
      optimized_traj_points[end_idx].pose, optimized_traj_points[end_idx + 1].pose);
    ++end_idx;
  }

  const geometry_utils::CircleFootprintRoadChecker road(
    planner_data.left_bound, planner_data.right_bound);
  // 3. assign zero velocity to the first point being outside the drivable area
  const auto first_outside_idx = [&]() -> std::optional<size_t> {
    for (size_t i = ego_idx; i < end_idx; ++i) {
      const auto & traj_point = optimized_traj_points.at(i);

      // check if the footprint is outside the drivable area
      const bool is_outside = outside_drivable_area_check_footprint_ == "vehicle_circles"
        ? road.isOutside(
            traj_point.pose,
            mpt_optimizer_ptr_->getVehicleCircleRadii(),
            mpt_optimizer_ptr_->getVehicleCircleLongitudinalOffsets())
        : geometry_utils::isOutsideDrivableAreaFromRectangleFootprint(
            traj_point.pose, planner_data.left_bound, planner_data.right_bound, vehicle_info_,
            use_footprint_polygon_for_outside_drivable_area_check_);

      if (is_outside) {
        return i;
      }
    }
    return std::nullopt;
  }();

  if (first_outside_idx) {
    if (std::getenv("AJOUNICE_PO_CIRCLE_DIAG") != nullptr && !debug_data_ptr_->ref_points.empty()) {
      static size_t reports = 0;
      if (reports++ < 4) {
        const auto & outside_pose = optimized_traj_points.at(*first_outside_idx).pose;
        const auto nearest = std::min_element(
          debug_data_ptr_->ref_points.begin(), debug_data_ptr_->ref_points.end(),
          [&](const auto & a, const auto & b) {
            return autoware_utils::calc_distance2d(a.pose, outside_pose) <
                   autoware_utils::calc_distance2d(b.pose, outside_pose);
          });
        const size_t ref_index = std::distance(debug_data_ptr_->ref_points.begin(), nearest);
        std::cerr << "[PO bound-check] output_pose=(" << outside_pose.position.x << ","
                  << outside_pose.position.y << ") ref_index=" << ref_index << " ref_pose=("
                  << nearest->pose.position.x << "," << nearest->pose.position.y
                  << ") ref_bounds=(" << nearest->bounds.lower_bound << ","
                  << nearest->bounds.upper_bound << ") optimization_failed="
                  << is_optimization_failed_ << " optimized_state=("
                  << nearest->optimized_kinematic_state.lat << ","
                  << nearest->optimized_kinematic_state.yaw << ")" << std::endl;
        for (size_t circle = 0; circle < nearest->bounds_on_constraints.size(); ++circle) {
          const auto & bounds = nearest->bounds_on_constraints.at(circle);
          std::cerr << "[PO bound-check] circle=" << circle << " qp_bounds=("
                    << bounds.lower_bound << "," << bounds.upper_bound << ") beta="
                    << nearest->beta.at(circle) << std::endl;
        }
      }
    }
    debug_data_ptr_->stop_pose_by_drivable_area = optimized_traj_points.at(*first_outside_idx).pose;
    const auto stop_idx = [&]() {
      const auto dist =
        autoware::motion_utils::calcSignedArcLength(optimized_traj_points, 0, *first_outside_idx);
      const auto dist_with_margin = dist - vehicle_stop_margin_outside_drivable_area_;
      const auto first_outside_idx_with_margin =
        autoware::motion_utils::insertTargetPoint(0, dist_with_margin, optimized_traj_points);
      if (first_outside_idx_with_margin) {
        return *first_outside_idx_with_margin;
      }
      return *first_outside_idx;
    }();

    publishVirtualWall(optimized_traj_points.at(stop_idx).pose);

    if (enable_outside_drivable_area_stop_) {
      for (size_t i = stop_idx; i < optimized_traj_points.size(); ++i) {
        optimized_traj_points.at(i).longitudinal_velocity_mps = 0.0;
      }
    }
  } else {
    debug_data_ptr_->stop_pose_by_drivable_area = std::nullopt;
  }
}

void PathOptimizer::publishVirtualWall(const geometry_msgs::Pose & stop_pose) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  auto virtual_wall_marker = autoware::motion_utils::createStopVirtualWallMarker(
    stop_pose, "outside drivable area", now(), 0, vehicle_info_.max_longitudinal_offset_m);
  if (!enable_outside_drivable_area_stop_) {
    virtual_wall_marker.markers.front().color =
      autoware_utils::create_marker_color(0.0, 1.0, 0.0, 0.5);
  }

  virtual_wall_pub_->publish(virtual_wall_marker);
}

void PathOptimizer::publishDebugMarkerOfOptimization(
  const std::vector<TrajectoryPoint> & traj_points) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  if (!enable_pub_debug_marker_) {
    return;
  }

  // debug marker
  time_keeper_->start_track("getDebugMarker");
  const auto debug_marker =
    getDebugMarker(*debug_data_ptr_, traj_points, vehicle_info_, enable_pub_extra_debug_marker_);
  time_keeper_->end_track("getDebugMarker");

  time_keeper_->start_track("publishDebugMarker");
  debug_markers_pub_->publish(debug_marker);
  time_keeper_->end_track("publishDebugMarker");
}

std::vector<TrajectoryPoint> PathOptimizer::extendTrajectory(
  const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<TrajectoryPoint> & optimized_traj_points) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto resample_output = [&](const std::vector<TrajectoryPoint> & points) {
    if (output_resampling_method_ == "gaussian") {
      return trajectory_utils::resampleTrajectoryPointsGaussian(
        points, traj_param_.output_delta_arc_length, output_gaussian_sigma_m_,
        debug_data_ptr_->ego_pose);
    }
    return trajectory_utils::resampleTrajectoryPoints(
      points, traj_param_.output_delta_arc_length, true);
  };

  const auto & joint_start_pose = optimized_traj_points.back().pose;

  // calculate end idx of optimized points on path points
  const size_t joint_start_traj_seg_idx =
    trajectory_utils::findEgoSegmentIndex(traj_points, joint_start_pose, ego_nearest_param_);

  // The MPT terminal samples match the input path. Retain every following
  // input sample; a 5--15 m gap discards road curvature and makes resampling
  // concentrate the missing turn at the far end of the join.
  size_t joint_end_traj_point_idx = joint_start_traj_seg_idx + 1;
  while (joint_end_traj_point_idx < traj_points.size() &&
         autoware_utils::calc_distance2d(
           traj_points.at(joint_end_traj_point_idx), joint_start_pose) < 1e-4) {
    ++joint_end_traj_point_idx;
  }
  if (joint_end_traj_point_idx >= traj_points.size()) {
    return resample_output(optimized_traj_points);
  }

  // calculate full trajectory points
  const auto full_traj_points = [&]() {
    auto extended_traj_points = std::vector<TrajectoryPoint>{
      traj_points.begin() + joint_end_traj_point_idx, traj_points.end()};

    if (!extended_traj_points.empty() && !optimized_traj_points.empty()) {
      // NOTE: Without this code, if optimized_traj_points's back is non zero velocity and
      // extended_traj_points' front
      //       is zero velocity, the zero velocity will be inserted in the whole joint trajectory.
      //       The input stop point will be inserted explicitly in the latter part.
      extended_traj_points.front().longitudinal_velocity_mps =
        optimized_traj_points.back().longitudinal_velocity_mps;
    }
    return concatVectors(optimized_traj_points, extended_traj_points);
  }();

  // resample trajectory points
  auto resampled_traj_points = resample_output(full_traj_points);

  // update stop velocity on joint
  for (size_t i = joint_start_traj_seg_idx + 1; i <= joint_end_traj_point_idx; ++i) {
    if (hasZeroVelocity(traj_points.at(i))) {
      if (i != 0 && !hasZeroVelocity(traj_points.at(i - 1))) {
        // Here is when current point is 0 velocity, but previous point is not 0 velocity.
        const auto & input_stop_pose = traj_points.at(i).pose;
        const size_t stop_seg_idx = trajectory_utils::findEgoSegmentIndex(
          resampled_traj_points, input_stop_pose, ego_nearest_param_);

        // calculate and insert stop pose on output trajectory
        trajectory_utils::insertStopPoint(resampled_traj_points, input_stop_pose, stop_seg_idx);
      }
    }
  }

  // debug_data_ptr_->extended_traj_points =
  //   extended_traj_points ? *extended_traj_points : std::vector<TrajectoryPoint>();
  return resampled_traj_points;
}

void PathOptimizer::publishDebugData(const Header & header) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // publish trajectories
  const auto debug_extended_traj =
    autoware::motion_utils::convertToTrajectory(debug_data_ptr_->extended_traj_points, header);
  debug_extended_traj_pub_->publish(debug_extended_traj);
}
std::optional<std::vector<TrajectoryPoint>> PathOptimizer::optimizeObstacleCandidates(
  const PlannerData & input_data, const double search_budget_ms)
{
  PlannerData data = input_data;
  data.obstacles = clusterStaticObstacles(
    input_data.obstacles, input_data.traj_points, obstacle_clustering_param_);
  if (data.obstacles.size() != input_data.obstacles.size()) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[MPT] Clustered static obstacles: raw=%zu clustered=%zu",
      input_data.obstacles.size(), data.obstacles.size());
  }
  // Bound exponential search explicitly. Never silently omit an obstacle.
  if (data.obstacles.size() > 6) {
    RCLCPP_ERROR(
      get_logger(), "[MPT] More than 6 spatially distinct static obstacles after clustering; "
      "safe stop required");
    return std::nullopt;
  }
  const auto original = mpt_optimizer_ptr_->history();
  const auto began = std::chrono::steady_clock::now();
  nlohmann::json trace, last_score;
  const bool tracing = obstacle_candidate_trace_enabled_;
  if (tracing) {
    trace = {{"header_ns", std::to_string(data.header.stamp.toNSec())},
      {"stamp", data.header.stamp.toSec()}, {"weight", obstacle_continuity_weight_},
      {"reset_reason", optimization_reset_reason_}, {"objects", data.obstacles.size()},
      {"history_present", static_cast<bool>(original.trajectory)},
      {"road_bounds_relaxed", data.relax_road_bounds},
      {"previous_present", static_cast<bool>(last_validated_output_)},
      {"candidates", nlohmann::json::array()}, {"attempts", nlohmann::json::array()}};
  }
  const auto trace_check = [&](const char * name, bool value) {
    if (tracing) trace["previous_checks"][name] = value;
    return value;
  };
  auto winner_history = original;
  std::optional<std::vector<TrajectoryPoint>> winner;
  std::optional<std::vector<TrajectoryPoint>> road_only_candidate;
  double road_only_cost = std::numeric_limits<double>::infinity();
  double road_only_contact_cost = std::numeric_limits<double>::infinity();
  size_t road_only_outside_count = std::numeric_limits<size_t>::max();
  const bool recovery_input_moving = !data.traj_points.empty() &&
    data.traj_points[trajectory_utils::findEgoIndex(
      data.traj_points, data.ego_pose, ego_nearest_param_)].longitudinal_velocity_mps > 0.1;
  double best_cost = std::numeric_limits<double>::infinity();
  double best_contact_cost = std::numeric_limits<double>::infinity();
  bool winner_is_previous = false;
  long winner_mask = -1;
  const geometry_utils::CircleFootprintRoadChecker road(data.left_bound, data.right_bound);
  const auto evaluate_cost = [&](const std::vector<TrajectoryPoint> & points, size_t ego,
                                 bool allow_road_violation = false)
    -> std::optional<double> {
    double cost = 0.0;
    double reference_cost = 0.0, obstacle_cost = 0.0, continuity_cost = 0.0;
    const size_t score_end = std::min(points.size(), ego + size_t{250});
    for (size_t i = ego; i < score_end; ++i) {
      const auto & pose = points.at(i).pose;
      const size_t reference = autoware::motion_utils::findNearestIndex(data.traj_points, pose.position);
      const double deviation = autoware_utils::calc_distance2d(pose, data.traj_points[reference].pose);
      cost += deviation * deviation;
      const double clearance_cost = mpt_optimizer_ptr_->obstacleClearanceWeight() *
        obstacleSoftViolation(pose, input_data.obstacles, vehicle_info_);
      cost += clearance_cost;
      if (tracing) {
        reference_cost += deviation * deviation;
        obstacle_cost += clearance_cost;
      }
      if (!allow_road_violation && !data.relax_road_bounds && i + 5 < score_end && road.isOutside(
            pose, mpt_optimizer_ptr_->getVehicleCircleRadii(),
            mpt_optimizer_ptr_->getVehicleCircleLongitudinalOffsets())) {
        return std::nullopt;
      }
      if (original.trajectory && !original.trajectory->empty()) {
        const size_t prev = autoware::motion_utils::findNearestIndex(*original.trajectory, pose.position);
        const double change = autoware_utils::calc_distance2d(pose, original.trajectory->at(prev).pose);
        const double previous_cost = obstacle_continuity_weight_ * change * change;
        cost += previous_cost;
        if (tracing) continuity_cost += previous_cost;
      }
    }
    const double count = std::max(size_t{1}, score_end - ego);
    if (tracing) last_score = {{"reference", reference_cost/count},
      {"clearance", obstacle_cost/count}, {"continuity", continuity_cost/count},
      {"total", cost/count}};
    return cost / count;
  };
  // A bounded search needs a freshly verified incumbent. Otherwise two equally
  // feasible sides alternate merely because only one was explored this cycle.
  // Use the same initial-state contract as the validated fallback. Still run
  // optimization each cycle; the incumbent permits bounded continuation work.
  if (last_validated_output_ && last_validated_output_->size() >= 3 &&
      !(recovery_state_.active() && !data.relax_road_bounds)) {
    const auto & previous = *last_validated_output_;
    const size_t ego = trajectory_utils::findEgoIndex(previous, data.ego_pose, ego_nearest_param_);
    const auto remaining = [&](const std::vector<TrajectoryPoint> & points, size_t start) {
      double distance = 0.0;
      for (size_t i = start + 1; i < points.size() && distance < 50.0; ++i) {
        distance += autoware_utils::calc_distance2d(points[i - 1], points[i]);
      }
      return std::min(50.0, distance);
    };
    const size_t input_ego = trajectory_utils::findEgoIndex(data.traj_points, data.ego_pose, ego_nearest_param_);
    const bool covers_horizon = remaining(previous, ego) + 1e-3 >= remaining(data.traj_points, input_ego);
    if (tracing) trace["previous_horizon"] = {{"remaining", remaining(previous, ego)},
      {"required", remaining(data.traj_points, input_ego)}};
    if (trace_check("covers_horizon", covers_horizon) &&
        trace_check("aligned", isAlignedWithEgoPose(previous[ego].pose, data.ego_pose)) &&
        trace_check("measured_inside_road", data.relax_road_bounds || measuredFootprintInsideRoad(data.ego_pose, data.left_bound, data.right_bound,
          vehicle_info_, mpt_optimizer_ptr_->hardRoadClearance())) &&
        trace_check("collision_free", !firstObstacleCollision(previous, ego, input_data.obstacles, vehicle_info_)) &&
        trace_check("curvature_valid", !firstCurvatureViolation(previous, ego, std::min(previous.size(), ego + size_t{250}),
          mpt_optimizer_ptr_->maximumCurvature()))) {
      if (const auto cost = evaluate_cost(previous, ego)) {
        winner = previous;
        best_cost = *cost;
        best_contact_cost = 0.0;
        winner_is_previous = true;
        if (tracing) trace["previous_score"] = last_score;
      }
    }
  }
  // Prepare the previous spatial seed once per callback, not for every mask.
  const bool has_previous_seed = !data.relax_road_bounds &&
    last_validated_output_ && last_validated_output_->size() >= 2;
  const auto previous_seed = has_previous_seed
    ? *last_validated_output_ : std::vector<TrajectoryPoint>{};
  bool prefer_previous_seed = false;
  if (has_previous_seed) {
    const auto ego = trajectory_utils::findEgoIndex(previous_seed, data.ego_pose, ego_nearest_param_);
    const auto & pose = previous_seed[ego].pose;
    const double yaw_error = tf2::getYaw(pose.orientation) - tf2::getYaw(data.ego_pose.orientation);
    prefer_previous_seed = autoware_utils::calc_distance2d(pose, data.ego_pose) <=
      traj_param_.output_delta_arc_length &&
      std::abs(std::atan2(std::sin(yaw_error), std::cos(yaw_error))) <=
        mpt_optimizer_ptr_->maximumCurvature()*traj_param_.output_delta_arc_length + 1e-3;
  }
  size_t feasible = 0;
  size_t attempts = 0;
  const bool have_validated_incumbent = winner.has_value();
  // Reserve 30 ms of the 100 ms publication period for postprocessing/scheduling.
  // This bounds retries, not the duration of an individual solver invocation.
  const double retry_budget_ms = have_validated_incumbent
    ? std::min(70.0, search_budget_ms) : search_budget_ms;
  double last_attempt_ms = 0.0;
  bool retry_budget_exhausted = false;
  const size_t mask_count = size_t{1} << data.obstacles.size();
  std::vector<bool> attempted_masks(mask_count, false);
  auto & search_cursor = data.relax_road_bounds
    ? relaxed_obstacle_search_cursor_ : obstacle_search_cursor_;
  size_t preferred_mask = search_cursor % mask_count;
  // A previous curve can still describe the intended pass side when its length
  // or current validity prevents direct reuse. Use that side only to order new
  // solves; every new candidate must satisfy the current measured-state, road,
  // obstacle and curvature checks before it can be selected.
  const bool prefer_previous_direction = has_previous_seed;
  if (prefer_previous_direction) {
    preferred_mask = 0;
    // Order side assignments by the previous spatial solution. This is only a
    // search hint: every new candidate uses the measured state and current scene.
    for (size_t o = 0; o < data.obstacles.size(); ++o) {
      const auto & polygon = data.obstacles[o].polygon.points;
      if (polygon.empty()) continue;
      geometry_msgs::Point center;
      for (const auto & vertex : polygon) { center.x += vertex.x; center.y += vertex.y; }
      center.x /= polygon.size(); center.y /= polygon.size();
      const auto index = autoware::motion_utils::findNearestIndex(previous_seed, center);
      const auto & pose = previous_seed[index].pose;
      const double yaw = tf2::getYaw(pose.orientation);
      const double lateral = -std::sin(yaw)*(center.x-pose.position.x) +
                              std::cos(yaw)*(center.y-pose.position.y);
      if (lateral < 0.0) preferred_mask |= size_t{1} << o;
    }
  }
  const size_t first_mask = preferred_mask;
  if (tracing) {
    trace["retained_incumbent"] = have_validated_incumbent;
    trace["prefer_previous_seed"] = prefer_previous_seed;
    trace["prefer_previous_direction"] = prefer_previous_direction;
    trace["first_mask"] = first_mask;
    trace["budget_ms"] = retry_budget_ms;
  }
  auto previous_candidate_began = began;
  size_t examined = 0;
    for (size_t order = 0; order < mask_count; ++order) {
      // Finish each candidate (including refinement/validation), but do not start
      // an unbounded sequence of candidates while the command grows stale.
      // Resume at the next side combination on the next cycle, using fresh data.
      const auto candidate_began = std::chrono::steady_clock::now();
      const double elapsed_ms = std::chrono::duration<double, std::milli>(
        candidate_began - began).count();
      const double previous_cost_ms = std::chrono::duration<double, std::milli>(
        candidate_began - previous_candidate_began).count();
      if (examined && elapsed_ms + previous_cost_ms >= retry_budget_ms) {
        if (tracing) trace["exit_reason"] = "side_search_budget";
        break;
      }
      previous_candidate_began = candidate_began;
      const size_t mask = (first_mask + order) % mask_count;
      search_cursor = (mask + 1) % mask_count;
      ++examined;
      PlannerData candidate = data;
      for (size_t o = 0; o < data.obstacles.size(); ++o) candidate.pass_left.push_back(mask & (size_t{1} << o));
      std::optional<std::vector<TrajectoryPoint>> trajectory;
      std::vector<TrajectoryPoint> checked_trajectory;
      size_t ego = 0;
      bool collision_free = false;
      std::optional<double> checked_cost;
      double checked_contact_cost = 0.0;
      // Start from the previous spatial solution during an ongoing turn. Retry
      // the route reference only when needed; validate against the complete scene.
      const size_t seeds = has_previous_seed ? 2 : 1;
      for (size_t seed = 0; seed < seeds && !collision_free && !retry_budget_exhausted; ++seed) {
      const bool use_previous_seed = has_previous_seed && (prefer_previous_seed ? seed == 0 : seed == 1);
      candidate.traj_points = use_previous_seed ? previous_seed : data.traj_points;
      for (size_t iteration = 0; iteration < 3; ++iteration) {
        const auto attempt_began = std::chrono::steady_clock::now();
        const double spent_ms = std::chrono::duration<double, std::milli>(attempt_began-began).count();
        if (attempts && spent_ms + last_attempt_ms >= retry_budget_ms) {
          retry_budget_exhausted = true;
          if (tracing) trace["exit_reason"] = "solver_retry_budget";
          break;
        }
        mpt_optimizer_ptr_->restoreHistory(
          !use_previous_seed && iteration == 0 ? original : MPTOptimizer::History{});
        ++attempts;
        attempted_masks[mask] = true;
        trajectory = mpt_optimizer_ptr_->optimizeTrajectory(candidate);
        last_attempt_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now()-attempt_began).count();
        if (tracing) trace["attempts"].push_back({{"mask", mask}, {"seed_previous", use_previous_seed},
          {"iteration", iteration}, {"solved", trajectory.has_value()},
          {"reject", mpt_optimizer_ptr_->lastRejectionReason()}, {"solve_ms", last_attempt_ms}});
        if (std::getenv("AJOUNICE_OBSTACLE_DIAG")) {
          RCLCPP_INFO(get_logger(), "[MPT candidate] t=%.3f mask=%zu iteration=%zu solved=%d",
            data.header.stamp.toSec(), mask, iteration, static_cast<int>(trajectory.has_value()));
        }
        if (!trajectory || trajectory->size() < 2) break;
        // Extension resamples positions and reconstructs yaw. Validate that exact geometry,
        // then relinearize around it if the first linear bicycle solution cuts a corner.
        auto publish_prefix = *trajectory;
        // Match optimizeTrajectory's velocity/stop insertion and backward crop
        // before extending. Cropping changes the resampling phase and its yaw.
        applyInputVelocity(publish_prefix, data.traj_points, data.ego_pose);
        checked_trajectory = extendTrajectory(data.traj_points, publish_prefix);
        ego = trajectory_utils::findEgoIndex(checked_trajectory, data.ego_pose, ego_nearest_param_);
        const auto prefix_end = autoware::motion_utils::findNearestIndex(
          checked_trajectory, trajectory->back().pose.position);
        const auto collision = firstObstacleCollision(
          checked_trajectory, ego, input_data.obstacles, vehicle_info_);
        const auto escape = collision && data.allow_initial_obstacle_overlap
          ? autoware::motion_utils::checkStaticObstacleEscape(
              checked_trajectory, data.ego_pose, input_data.obstacles, vehicle_info_)
          : autoware::motion_utils::StaticObstacleEscapeResult{};
        const bool obstacle_valid = !collision || escape.valid;
        checked_contact_cost = escape.contact_cost;
        // The spatial resampling can rotate the tangent away from the QP's
        // fixed initial yaw. A newly accepted curve must meet the same measured
        // state contract as a retained curve; otherwise relinearize it first.
        if (isAlignedWithEgoPose(checked_trajectory.at(ego).pose, data.ego_pose) &&
            obstacle_valid &&
            !firstCurvatureViolation(checked_trajectory, ego, prefix_end,
              mpt_optimizer_ptr_->maximumCurvature()) &&
            (checked_cost = evaluate_cost(checked_trajectory, ego))) {
          collision_free = true;
          if (tracing) trace["attempts"].back()["postcheck"] = "accepted";
          break;
        }
        // Preserve a candidate rejected ONLY by the road check. It cannot win
        // over any ordinary solution, and is released only after every side
        // assignment actually received a solve in the current scene.
        if (recovery_state_.active() && recovery_permitted_ && data.relax_road_bounds &&
            enable_obstacle_fallback_ && recovery_input_moving && !checked_cost && obstacle_valid &&
            isAlignedWithEgoPose(checked_trajectory.at(ego).pose, data.ego_pose) &&
            !firstCurvatureViolation(checked_trajectory, ego, checked_trajectory.size(),
              mpt_optimizer_ptr_->maximumCurvature())) {
          const auto relaxed_cost = evaluate_cost(checked_trajectory, ego, true);
          size_t outside_count = 0;
          for (size_t i = ego; i + 5 < std::min(checked_trajectory.size(), ego+size_t{250}); ++i) {
            outside_count += road.isOutside(checked_trajectory[i].pose,
              mpt_optimizer_ptr_->getVehicleCircleRadii(),
              mpt_optimizer_ptr_->getVehicleCircleLongitudinalOffsets());
          }
          if (relaxed_cost && std::tie(checked_contact_cost, outside_count, *relaxed_cost) <
              std::tie(road_only_contact_cost, road_only_outside_count, road_only_cost)) {
            road_only_candidate = checked_trajectory;
            road_only_cost = *relaxed_cost;
            road_only_contact_cost = checked_contact_cost;
            road_only_outside_count = outside_count;
          }
        }
        if (tracing) {
          const auto bend = firstCurvatureViolation(checked_trajectory, ego, prefix_end,
            mpt_optimizer_ptr_->maximumCurvature());
          trace["attempts"].back()["postcheck"] = {
            {"aligned", isAlignedWithEgoPose(checked_trajectory.at(ego).pose, data.ego_pose)},
            {"collision_index", collision ? static_cast<long>(*collision) : -1L},
            {"initial_overlap_escape", escape.valid},
            {"curvature_index", bend ? static_cast<long>(*bend) : -1L},
            {"score_valid", checked_cost.has_value()}};
        }
        if (std::getenv("AJOUNICE_OBSTACLE_DIAG")) {
          const auto hit = collision;
          const auto bend = firstCurvatureViolation(checked_trajectory, ego, prefix_end,
            mpt_optimizer_ptr_->maximumCurvature());
          RCLCPP_INFO(get_logger(),
            "[MPT postcheck] t=%.3f mask=%zu seed=%zu iteration=%zu ego=%zu prefix_end=%zu collision=%ld curvature=%ld aligned=%d",
            data.header.stamp.toSec(), mask, seed, iteration, ego, prefix_end,
            hit ? static_cast<long>(*hit) : -1L, bend ? static_cast<long>(*bend) : -1L,
            static_cast<int>(isAlignedWithEgoPose(checked_trajectory.at(ego).pose, data.ego_pose)));
        }
        // Relinearizing the same prefix cannot repair a collision after the QP.
        // Coverage is chosen before solving; do not spend three solves on a suffix.
        if (collision && *collision > prefix_end) break;
        candidate.traj_points = checked_trajectory;
      }
      }
      if (!collision_free) {
        if (tracing) trace["candidates"].push_back({{"mask", mask}, {"feasible", false}});
        continue;
      }
      const auto & cost = checked_cost;
      if (!cost) continue;
      ++feasible;
      if (tracing) trace["candidates"].push_back({{"mask", mask}, {"feasible", true},
        {"score", last_score}, {"incumbent_cost", best_cost},
        {"incumbent_previous", winner_is_previous},
        {"replace", winner_is_previous || *cost < best_cost}});
      // The retained path is a fallback, not a competing new solution. Prefer
      // a freshly optimized, fully validated trajectory anchored to current ego.
      if (winner_is_previous || std::tie(checked_contact_cost, *cost) < std::tie(best_contact_cost, best_cost)) {
        best_cost = *cost; winner = checked_trajectory; winner_history = mpt_optimizer_ptr_->history();
        best_contact_cost = checked_contact_cost;
        winner_is_previous = false;
        winner_mask = static_cast<long>(mask);
      }
      // Continue receding-horizon optimization each cycle, but finish once the
      // continuation has produced a fully validated new solution. If it fails,
      // all other side assignments remain available within the search budget.
      if (prefer_previous_direction && mask == preferred_mask) {
        if (tracing) trace["exit_reason"] = "preferred_continuation_succeeded";
        break;
      }
    }
  normal_obstacle_search_complete_ = std::all_of(
    attempted_masks.begin(), attempted_masks.end(), [](bool tried) { return tried; });
  if (recovery_state_.active() && recovery_permitted_ && data.relax_road_bounds &&
      normal_obstacle_search_complete_ && road_only_candidate &&
      (!winner || road_only_contact_cost < best_contact_cost)) {
    winner = std::move(road_only_candidate);
    winner_history = original;
    best_cost = road_only_cost;
    using_recovery_geometry_ = true;
    if (tracing) trace["recovery"] = {{"contact_escape", false},
      {"outside_road", true}, {"source", "road_only_rejected_candidate"}};
  }
  mpt_optimizer_ptr_->restoreHistory(winner ? winner_history : original);
  // Return exactly the postprocessed geometry that passed candidate checks.
  // Re-extending/resampling it later would invalidate that validation contract.
  using_final_geometry_ = winner.has_value();
  selected_previous_ = winner.has_value() && winner_is_previous;
  if (tracing) {
    trace["winner_mask"] = winner_mask;
    trace["winner_previous"] = winner_is_previous;
    trace["winner_present"] = winner.has_value();
    trace["best_cost"] = best_cost;
    trace["examined"] = examined;
    trace["feasible"] = feasible;
    trace["search_ms"] = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now()-began).count();
    obstacle_candidate_trace_ = trace.dump();
  }
  if (winner_is_previous) {
    applyInputVelocity(*winner, data.traj_points, data.ego_pose);

  }
  mpt_optimizer_ptr_->publishAcceptedDebug(data.header);
  if (std::getenv("AJOUNICE_OBSTACLE_DIAG")) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "[MPT] Static obstacle candidates: objects=%zu examined=%zu/%zu feasible=%zu selected=%d retained=%d attempts=%zu",
      data.obstacles.size(), examined, mask_count, feasible, static_cast<int>(winner.has_value()),
      static_cast<int>(winner_is_previous), attempts);
  }
  if (examined < mask_count && feasible == 0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "[MPT] Candidate search budget (%.0fms) defers remaining sides to next cycle",
      retry_budget_ms);
  }
  return winner;
}

bool PathOptimizer::stopBeforeObstacle(
  const PlannerData & data, std::vector<TrajectoryPoint> & points) const
{
  if (points.size() < 2 || data.obstacles.empty()) return false;
  const size_t ego = trajectory_utils::findEgoIndex(points, data.ego_pose, ego_nearest_param_);
  auto collision = firstObstacleCollision(points, ego, data.obstacles, vehicle_info_);
  if (collision && data.allow_initial_obstacle_overlap &&
      autoware::motion_utils::checkStaticObstacleEscape(
        points, data.ego_pose, data.obstacles, vehicle_info_).valid) collision.reset();
  const size_t horizon = static_cast<size_t>(
    mpt_optimizer_ptr_->getNumberOfPoints() * mpt_optimizer_ptr_->getDeltaArcLength() /
    traj_param_.output_delta_arc_length);
  const auto curvature = firstCurvatureViolation(points, ego, std::min(points.size(), ego + horizon),
    mpt_optimizer_ptr_->maximumCurvature());
  if (curvature && (!collision || *curvature < *collision)) collision = curvature;
  if (!collision) return false;
  for (size_t i = ego; i <= *collision; ++i) {
    // A zero-speed suffix already forbids traversal of this geometry.
    if (std::abs(points[i].longitudinal_velocity_mps) < 1e-4) return false;
  }
  double clear = 0.0;
  for (size_t i = ego + 1; i <= *collision; ++i) {
    clear += autoware_utils::calc_distance2d(points[i-1].pose, points[i].pose);
  }
  const double stop = std::max(0.0, clear - 1.0);
  const bool emergency = data.ego_vel * data.ego_vel > 2.0 * 3.0 * stop;
  double distance = 0.0;
  for (size_t i = ego; i < points.size(); ++i) {
    if (i > ego) distance += autoware_utils::calc_distance2d(points[i-1].pose, points[i].pose);
    points[i].longitudinal_velocity_mps = emergency ? 0.0 : std::min<double>(
      points[i].longitudinal_velocity_mps, std::sqrt(6.0 * std::max(0.0, stop - distance)));
  }
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
    "[MPT] Final geometry unsafe (footprint/curvature): clear=%.2f required=%.2f emergency=%d",
    clear, data.ego_vel * data.ego_vel / 6.0, static_cast<int>(emergency));
  return true;
}

}  // namespace autoware::path_optimizer
