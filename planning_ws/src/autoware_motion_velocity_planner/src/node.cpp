#include "full_native_support.hpp"
// Copyright 2024 Tier IV, Inc.
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

#include "node.hpp"
#include <autoware/route_handler/route_handler.hpp>

#include "autoware/motion_velocity_planner_common/utils.hpp"

#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/velocity_smoother/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.hpp>
#include <autoware/velocity_smoother/trajectory_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_pcl/transforms.hpp>
#include <autoware_utils_rclcpp/parameter.hpp>
#include <autoware_utils_system/stop_watch.hpp>
#include <ros/time.h>
#include <tf2_eigen/tf2_eigen.h>

#include <autoware_internal_debug_msgs/Float64Stamped.h>
#include <autoware_planning_msgs/TrajectoryPoint.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner
{
MotionVelocityPlannerNode::MotionVelocityPlannerNode(ros::NodeHandle private_node)
: full_native::Node(std::move(private_node)), tf_buffer_(), tf_listener_(tf_buffer_),
  planner_data_(std::make_shared<PlannerData>(*this))
{
  sub_trajectory_=subscribe("input/trajectory",1,&MotionVelocityPlannerNode::on_trajectory,this);
  sub_lanelet_map_=subscribe("input/vector_map",10,&MotionVelocityPlannerNode::on_lanelet_map,this);
  srv_load_plugin_=advertiseService("service/load_plugin",&MotionVelocityPlannerNode::on_load_plugin,this);
  srv_unload_plugin_=advertiseService("service/unload_plugin",&MotionVelocityPlannerNode::on_unload_plugin,this);
  autoware_utils_debug::ros1::PublisherFactory factory;
  trajectory_pub_=factory.advertise<Trajectory>(*this,"output/trajectory",1);
  velocity_limit_pub_=advertise<VelocityLimit>("output/velocity_limit",1,true);
  clear_velocity_limit_pub_=advertise<VelocityLimitClearCommand>("output/clear_velocity_limit",1,true);
  processing_time_publisher_=std::make_shared<autoware_utils_debug::DebugPublisher>(debug_context_,"debug");
  debug_viz_pub_=advertise<visualization_msgs::MarkerArray>("debug/markers",1);
  debug_processed_pointcloud_pub_=advertise<sensor_msgs::PointCloud2>("debug/processed_pointcloud",1);
  smooth_velocity_before_planning_=declare_parameter<bool>("smooth_velocity_before_planning");
  set_velocity_smoother_params();
  for(const auto & name:declare_parameter<std::vector<std::string>>("launch_modules")) {
    if(name.empty()) break;
    planner_manager_.load_module_plugin(*this,name);
  }
  last_parameters_=full_native::parameters(*this);
  parameter_timer_=createTimer(ros::Duration(0.2),[this](const ros::TimerEvent &){poll_parameters();});
  logger_configure_=std::make_unique<autoware_utils_logging::LoggerLevelConfigure>(*this);
}

bool MotionVelocityPlannerNode::on_load_plugin(LoadPlugin::Request & request,LoadPlugin::Response & response)
{
  std::unique_lock<std::mutex> lock(mutex_);
  try { planner_manager_.load_module_plugin(*this,request.plugin_name); response.success=planner_manager_.has_module(request.plugin_name); }
  catch(const std::exception & error) { ROS_ERROR("Plugin load failed: %s",error.what()); response.success=false; }
  return true;
}
bool MotionVelocityPlannerNode::on_unload_plugin(UnloadPlugin::Request & request,UnloadPlugin::Response & response)
{
  std::unique_lock<std::mutex> lock(mutex_);
  response.success=planner_manager_.has_module(request.plugin_name);
  planner_manager_.unload_module_plugin(*this,request.plugin_name); return true;
}
void MotionVelocityPlannerNode::poll_parameters()
{
  const auto current=full_native::parameters(*this);
  ParameterMap changed;
  for(const auto & item:current) {
    const auto old=last_parameters_.find(item.first);
    if(old==last_parameters_.end() || old->second!=item.second) changed.insert(item);
    if(old!=last_parameters_.end() && old->second.getType()!=item.second.getType()) {
      ROS_ERROR_THROTTLE(1.0,"MVP parameter type change rejected"); return;
    }
  }
  try { if(!changed.empty() && on_set_param(changed)) last_parameters_=current; }
  catch(const std::exception & error) { ROS_ERROR_THROTTLE(1.0,"MVP parameter update rejected: %s",error.what()); }
}

// NOTE: argument planner_data must not be referenced for multithreading
bool MotionVelocityPlannerNode::update_planner_data(
  std::map<std::string, double> & processing_times,
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & input_traj_points)
{
  auto clock = *get_clock();
  auto is_ready = true;
  const auto check_with_log = [&](const auto ptr, const auto & log, const bool is_required = true) {
    if (!is_required) {
      return false;
    }

    constexpr auto throttle_duration = 3000;  // [ms]
    if (!ptr) {
      RCLCPP_INFO_THROTTLE(get_logger(), clock, throttle_duration, "%s", log);
      is_ready = false;
      return false;
    }
    return true;
  };

  const auto required_subscriptions = planner_manager_.getRequiredSubscriptions();

  autoware_utils_system::StopWatch<std::chrono::milliseconds> sw;
  const auto ego_state_ptr = sub_vehicle_odometry_.take_data();
  if (check_with_log(ego_state_ptr, "Waiting for current odometry"))
    planner_data_->current_odometry = *ego_state_ptr;
  processing_times["update_planner_data.odom"] = sw.toc(true);

  const auto ego_accel_ptr = sub_acceleration_.take_data();
  if (check_with_log(ego_accel_ptr, "Waiting for current acceleration"))
    planner_data_->current_acceleration = *ego_accel_ptr;
  processing_times["update_planner_data.accel"] = sw.toc(true);

  const auto predicted_objects_ptr = sub_predicted_objects_.take_data();
  if (check_with_log(
        predicted_objects_ptr, "Waiting for predicted objects",
        required_subscriptions.predicted_objects))
    planner_data_->process_predicted_objects(*predicted_objects_ptr);
  processing_times["update_planner_data.pred_obj"] = sw.toc(true);

  const auto no_ground_pointcloud_ptr = sub_no_ground_pointcloud_.take_data();
  if (check_with_log(
        no_ground_pointcloud_ptr, "Waiting for pointcloud",
        required_subscriptions.no_ground_pointcloud)) {
    sw.tic("process_no_ground_pointcloud");
    auto no_ground_pointcloud = process_no_ground_pointcloud(no_ground_pointcloud_ptr);
    processing_times["update_planner_data.pointcloud.affine_transform"] =
      sw.toc("process_no_ground_pointcloud");
    sw.tic("preprocess_pointcloud");
    if (!no_ground_pointcloud) is_ready=false;
    if (no_ground_pointcloud) {
      planner_data_->no_ground_pointcloud.preprocess_pointcloud(
        std::move(*no_ground_pointcloud), input_traj_points, planner_data_->current_odometry,
        planner_data_->calculate_min_deceleration_distance(0.0).value_or(0.0),
        planner_data_->vehicle_info_, planner_data_->trajectory_polygon_collision_check,
        planner_data_->ego_nearest_dist_threshold, planner_data_->ego_nearest_yaw_threshold);
    }
    processing_times["update_planner_data.pointcloud.preprocess_for_obstacle_*_modules"] =
      sw.toc("preprocess_pointcloud");
  }
  processing_times["update_planner_data.pointcloud"] = sw.toc(true);

  const auto occupancy_grid_ptr = sub_occupancy_grid_.take_data();
  if (check_with_log(
        occupancy_grid_ptr, "Waiting for the occupancy grid",
        required_subscriptions.occupancy_grid_map))
    planner_data_->occupancy_grid_map = *occupancy_grid_ptr;
  processing_times["update_planner_data.occ_grid"] = sw.toc(true);

  // here we use bitwise operator to not short-circuit the logging messages
  is_ready &= check_with_log(map_ptr_, "Waiting for the map");
  processing_times["update_planner_data.map"] = sw.toc(true);
  is_ready &= check_with_log(
    planner_data_->velocity_smoother_, "Waiting for the initialization of the velocity smoother");
  processing_times["update_planner_data.smoother"] = sw.toc(true);

  // is driving forward
  const auto is_driving_forward =
    autoware::motion_utils::isDrivingForwardWithTwist(input_traj_points);
  if (is_driving_forward) {
    planner_data_->is_driving_forward = is_driving_forward.value();
  }
  processing_times["update_planner_data.is_driving_forward"] = sw.toc(true);

  // optional data
  const auto traffic_signals_ptr = sub_traffic_signals_.take_data();
  // NOTE: required_subscriptions.traffic_signals is not used since is_ready is not updated here.
  if (traffic_signals_ptr) process_traffic_signals(traffic_signals_ptr);
  processing_times["update_planner_data.traffic_lights"] = sw.toc(true);

  return is_ready;
}

std::optional<pcl::PointCloud<pcl::PointXYZ>>
MotionVelocityPlannerNode::process_no_ground_pointcloud(
  const sensor_msgs::PointCloud2::ConstPtr msg)
{
  geometry_msgs::TransformStamped transform;
  const bool is_pcl_time_valid = (this->get_clock()->now() - ros::Time(msg->header.stamp)) <
                                 ros::Duration(1.0);

  if (
    is_pcl_time_valid && tf_buffer_.canTransform("map", msg->header.frame_id, msg->header.stamp)) {
    transform = tf_buffer_.lookupTransform(
      "map", msg->header.frame_id, msg->header.stamp, ros::Duration(0.05));
  } else if (tf_buffer_.canTransform("map", msg->header.frame_id, ros::Time(0))) {
    transform = tf_buffer_.lookupTransform("map", msg->header.frame_id, ros::Time(0));
    RCLCPP_DEBUG(get_logger(), "pcl time is invalid, using ros::Time(0)");
  } else {
    RCLCPP_WARN(get_logger(), "no transform found for no_ground_pointcloud");
    return std::nullopt;
  }

  pcl::PointCloud<pcl::PointXYZ> pc_input;
  pcl::fromROSMsg(*msg, pc_input);

  const Eigen::Affine3f affine = tf2::transformToEigen(transform.transform).cast<float>();
  pcl::PointCloud<pcl::PointXYZ> pc_transformed;
  if (!pc_input.empty()) autoware_utils_pcl::transform_pointcloud(pc_input, pc_transformed, affine);

  pc_transformed.header = pc_input.header;
  pc_transformed.header.frame_id = "map";

  return pc_transformed;
}

void MotionVelocityPlannerNode::set_velocity_smoother_params()
{
  planner_data_->velocity_smoother_ =
    std::make_shared<autoware::velocity_smoother::AnalyticalJerkConstrainedSmoother>(smoother_parameters_);
}

void MotionVelocityPlannerNode::on_lanelet_map(
  const autoware_map_msgs::LaneletMapBin::ConstPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  map_ptr_ = msg;
  has_received_map_ = true;
}

void MotionVelocityPlannerNode::process_traffic_signals(
  const autoware_perception_msgs::TrafficLightGroupArray::ConstPtr msg)
{
  // clear previous observation
  planner_data_->traffic_light_id_map_raw_.clear();
  const auto traffic_light_id_map_last_observed_old =
    planner_data_->traffic_light_id_map_last_observed_;
  planner_data_->traffic_light_id_map_last_observed_.clear();
  for (const auto & signal : msg->traffic_light_groups) {
    TrafficSignalStamped traffic_signal;
    traffic_signal.stamp = full_native::time(msg->stamp);
    traffic_signal.signal = signal;
    planner_data_->traffic_light_id_map_raw_[signal.traffic_light_group_id] = traffic_signal;
    const bool is_unknown_observation =
      std::any_of(signal.elements.begin(), signal.elements.end(), [](const auto & element) {
        return element.color == autoware_perception_msgs::TrafficLightElement::UNKNOWN;
      });
    // if the observation is UNKNOWN and past observation is available, only update the timestamp
    // and keep the body of the info
    const auto old_data =
      traffic_light_id_map_last_observed_old.find(signal.traffic_light_group_id);
    if (is_unknown_observation && old_data != traffic_light_id_map_last_observed_old.end()) {
      // copy last observation
      planner_data_->traffic_light_id_map_last_observed_[signal.traffic_light_group_id] =
        old_data->second;
      // update timestamp
      planner_data_->traffic_light_id_map_last_observed_[signal.traffic_light_group_id].stamp =
        full_native::time(msg->stamp);
    } else {
      planner_data_->traffic_light_id_map_last_observed_[signal.traffic_light_group_id] =
        traffic_signal;
    }
  }
}

void MotionVelocityPlannerNode::on_trajectory(
  const autoware_planning_msgs::Trajectory::ConstPtr input_trajectory_msg)
{
  std::unique_lock<std::mutex> lk(mutex_);

  autoware_utils_system::StopWatch<std::chrono::milliseconds> stop_watch;
  std::map<std::string, double> processing_times;
  stop_watch.tic("Total");

  if (!update_planner_data(processing_times, input_trajectory_msg->points)) {
    return;
  }
  processing_times["update_planner_data"] = stop_watch.toc(true);

  if (input_trajectory_msg->points.empty()) {
    RCLCPP_WARN(get_logger(), "Input trajectory message is empty");
    return;
  }

  if (has_received_map_) {
    planner_data_->route_handler = std::make_shared<route_handler::RouteHandler>(*map_ptr_);
    has_received_map_ = false;
    processing_times["make_RouteHandler"] = stop_watch.toc(true);
  }

  autoware::motion_velocity_planner::TrajectoryPoints input_trajectory_points{
    input_trajectory_msg->points.begin(), input_trajectory_msg->points.end()};
  autoware_planning_msgs::Trajectory output_trajectory_msg;
  try { output_trajectory_msg=generate_trajectory(input_trajectory_points,processing_times); }
  catch(const std::exception & error) { ROS_ERROR("MVP planning failed: %s",error.what()); return; }
  output_trajectory_msg.header = input_trajectory_msg->header;
  processing_times["generate_trajectory"] = stop_watch.toc(true);

  std::optional<sensor_msgs::PointCloud2> debug_processed_pointcloud_msg;
  if (
    debug_processed_pointcloud_pub_.getNumSubscribers() > 0 &&
    planner_data_->no_ground_pointcloud.preprocess_params_.downsample_by_voxel_grid
      .enable_downsample) {
    if (planner_data_->no_ground_pointcloud.has_processed_pointcloud()) {
      try {
        sensor_msgs::PointCloud2 output_pointcloud_msg;
        pcl::toROSMsg(
          planner_data_->no_ground_pointcloud.extract_clustered_points(), output_pointcloud_msg);
        debug_processed_pointcloud_msg = std::move(output_pointcloud_msg);
      } catch (const std::exception & error) {
        // A debug subscriber must never be able to terminate the required planning node.
        ROS_WARN_THROTTLE(
          1.0, "Skipping debug processed pointcloud publication: %s", error.what());
      }
    } else {
      ROS_DEBUG_THROTTLE(
        1.0, "Skipping debug processed pointcloud publication because pointcloud preprocessing "
        "has not run for the loaded planner modules");
    }
  }

  lk.unlock();

  trajectory_pub_->publish(output_trajectory_msg);

  if (debug_processed_pointcloud_msg) {
    debug_processed_pointcloud_pub_.publish(*debug_processed_pointcloud_msg);
    processing_times["publish_down_sampled_pointcloud"] = stop_watch.toc(true);
  }

  published_time_publisher_.publish_if_subscribed(
    *trajectory_pub_, output_trajectory_msg.header.stamp);
  processing_times["Total"] = stop_watch.toc("Total");
  processing_diag_publisher_.publish(processing_times);

  processing_time_publisher_->publish<autoware_internal_debug_msgs::Float64Stamped>(
    "processing_time_ms", processing_times["Total"]);
  processing_time_publisher_->publish<autoware_internal_debug_msgs::Float64Stamped>(
    "mvp_common/processing_time_ms",
    processing_times["Total"] - processing_times["plan_velocities"]);
}

autoware::motion_velocity_planner::TrajectoryPoints MotionVelocityPlannerNode::smooth_trajectory(
  const autoware::motion_velocity_planner::TrajectoryPoints & trajectory_points,
  const std::shared_ptr<autoware::motion_velocity_planner::PlannerData> & planner_data) const
{
  const geometry_msgs::Pose current_pose = planner_data->current_odometry.pose.pose;
  const double v0 = planner_data->current_odometry.twist.twist.linear.x;
  const double a0 = planner_data->current_acceleration.accel.accel.linear.x;
  const auto & smoother = planner_data->velocity_smoother_;

  const auto traj_lateral_acc_filtered =
    smoother->applyLateralAccelerationFilter(trajectory_points);

  const auto traj_steering_rate_limited =
    smoother->applySteeringRateLimit(traj_lateral_acc_filtered, false, smoother->getBaseParam().sample_ds);

  // Resample trajectory with ego-velocity based interval distances
  auto traj_resampled = traj_steering_rate_limited;
  const size_t traj_resampled_closest =
    autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
      traj_resampled, current_pose, planner_data->ego_nearest_dist_threshold,
      planner_data->ego_nearest_yaw_threshold);
  std::vector<autoware::motion_velocity_planner::TrajectoryPoints> debug_trajectories;
  // Clip trajectory from closest point
  autoware::motion_velocity_planner::TrajectoryPoints clipped;
  autoware::motion_velocity_planner::TrajectoryPoints traj_smoothed;
  clipped.insert(
    clipped.end(), std::next(traj_resampled.begin(), static_cast<int64_t>(traj_resampled_closest)),
    traj_resampled.end());
  if (!smoother->apply(v0, a0, clipped, traj_smoothed, debug_trajectories, false)) {
    throw std::runtime_error("failed to smooth");
  }
  return traj_smoothed;
}

autoware_planning_msgs::Trajectory MotionVelocityPlannerNode::generate_trajectory(
  const autoware::motion_velocity_planner::TrajectoryPoints & input_trajectory_points,
  std::map<std::string, double> & processing_times)
{
  autoware_utils_system::StopWatch<std::chrono::milliseconds> stop_watch;
  autoware_planning_msgs::Trajectory output_trajectory_msg;
  output_trajectory_msg.points = {input_trajectory_points.begin(), input_trajectory_points.end()};

  stop_watch.tic("smooth");
  const auto smoothed_trajectory_points = [&]() {
    if (smooth_velocity_before_planning_) {
      return smooth_trajectory(input_trajectory_points, planner_data_);
    }
    return input_trajectory_points;
  }();
  processing_times["velocity_smoothing"] = stop_watch.toc("smooth");

  stop_watch.tic("resample");
  TrajectoryPoints resampled_smoothed_trajectory_points;
  // skip points that are too close together to make computation easier
  if (!smoothed_trajectory_points.empty()) {
    resampled_smoothed_trajectory_points.push_back(smoothed_trajectory_points.front());
    constexpr auto min_interval_squared = 0.5 * 0.5;  // TODO(Maxime): change to a parameter
    for (auto i = 1UL; i < smoothed_trajectory_points.size(); ++i) {
      const auto & p = smoothed_trajectory_points[i];
      const auto dist_to_prev_point = autoware_utils_geometry::calc_squared_distance2d(
        resampled_smoothed_trajectory_points.back(), p);
      if (dist_to_prev_point > min_interval_squared) {
        resampled_smoothed_trajectory_points.push_back(p);
      }
    }
  }
  processing_times["resample"] = stop_watch.toc("resample");
  stop_watch.tic("calculate_time_from_start");
  motion_utils::calculate_time_from_start(
    resampled_smoothed_trajectory_points, planner_data_->current_odometry.pose.pose.position);
  processing_times["calculate_time_from_start"] = stop_watch.toc("calculate_time_from_start");
  stop_watch.tic("plan_velocities");
  const auto planning_results = planner_manager_.plan_velocities(
    input_trajectory_points, resampled_smoothed_trajectory_points, planner_data_,
    processing_time_publisher_);
  processing_times["plan_velocities"] = stop_watch.toc("plan_velocities");

  for (const auto & planning_result : planning_results) {
    utils::applyPlanningResult(output_trajectory_msg.points, planning_result);
    if (planning_result.velocity_limit) {
      velocity_limit_pub_.publish(*planning_result.velocity_limit);
    }
    if (planning_result.velocity_limit_clear_command) {
      clear_velocity_limit_pub_.publish(*planning_result.velocity_limit_clear_command);
    }
  }

  return output_trajectory_msg;
}

bool MotionVelocityPlannerNode::on_set_param(
  const ParameterMap & parameters)
{
  using full_native::update_param;

  {
    std::unique_lock<std::mutex> lk(mutex_);  // for planner_manager_
    planner_manager_.update_module_parameters(parameters);
  }

  update_param(parameters, "smooth_velocity_before_planning", smooth_velocity_before_planning_);
  update_param(parameters, "ego_nearest_dist_threshold", planner_data_->ego_nearest_dist_threshold);
  update_param(parameters, "ego_nearest_yaw_threshold", planner_data_->ego_nearest_yaw_threshold);
  update_param(
    parameters, "trajectory_polygon_collision_check.decimate_trajectory_step_length",
    planner_data_->trajectory_polygon_collision_check.decimate_trajectory_step_length);
  update_param(
    parameters, "trajectory_polygon_collision_check.goal_extended_trajectory_length",
    planner_data_->trajectory_polygon_collision_check.goal_extended_trajectory_length);
  update_param(
    parameters,
    "trajectory_polygon_collision_check.consider_current_pose.enable_to_consider_current_pose",
    planner_data_->trajectory_polygon_collision_check.enable_to_consider_current_pose);
  update_param(
    parameters, "trajectory_polygon_collision_check.consider_current_pose.time_to_convergence",
    planner_data_->trajectory_polygon_collision_check.time_to_convergence);

  // set_velocity_smoother_params(); TODO(Maxime): fix update parameters of the velocity smoother

  return true;
}
}  // namespace autoware::motion_velocity_planner
