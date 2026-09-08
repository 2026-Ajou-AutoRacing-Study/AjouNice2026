// Copyright 2022-2024 TIER IV, Inc.
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

#include "obstacle_velocity_limiter_module.hpp"

#include "debug.hpp"
#include "map_utils.hpp"
#include "obstacle_velocity_limiter.hpp"
#include "parameters.hpp"
#include "trajectory_preprocessing.hpp"

#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/parameter.hpp>
#include <autoware_utils/ros/processing_time_publisher.hpp>
#include <autoware_utils/ros/update_param.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <autoware_utils_debug/ros1_adapters.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <boost/geometry.hpp>

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::motion_velocity_planner
{
namespace
{
bool as_double(const XmlRpc::XmlRpcValue & value, double & output)
{
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    output = static_cast<double>(value);
    return true;
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    output = static_cast<int>(value);
    return true;
  }
  return false;
}

bool as_int(const XmlRpc::XmlRpcValue & value, int & output)
{
  if (value.getType() != XmlRpc::XmlRpcValue::TypeInt) return false;
  output = static_cast<int>(value);
  return true;
}

bool as_bool(const XmlRpc::XmlRpcValue & value, bool & output)
{
  if (value.getType() != XmlRpc::XmlRpcValue::TypeBoolean) return false;
  output = static_cast<bool>(value);
  return true;
}

bool as_string(const XmlRpc::XmlRpcValue & value, std::string & output)
{
  if (value.getType() != XmlRpc::XmlRpcValue::TypeString) return false;
  output = static_cast<std::string>(value);
  return true;
}

bool as_string_array(const XmlRpc::XmlRpcValue & value, std::vector<std::string> & output)
{
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray) return false;
  std::vector<std::string> converted;
  converted.reserve(value.size());
  for (int i = 0; i < value.size(); ++i) {
    if (value[i].getType() != XmlRpc::XmlRpcValue::TypeString) return false;
    converted.push_back(static_cast<std::string>(value[i]));
  }
  output = std::move(converted);
  return true;
}

void warn_invalid_update(const std::string & name)
{
  ROS_WARN("Rejected invalid runtime parameter update for %s", name.c_str());
}
}  // namespace

void ObstacleVelocityLimiterModule::init(ros::NodeHandle & node, const std::string & module_name)
{
  module_name_ = module_name;
  node_owner_ = std::make_shared<ros::NodeHandle>(node);
  preprocessing_params_ = obstacle_velocity_limiter::PreprocessingParameters(node);
  projection_params_ = obstacle_velocity_limiter::ProjectionParameters(node);
  obstacle_params_ = obstacle_velocity_limiter::ObstacleParameters(node);
  velocity_params_ = obstacle_velocity_limiter::VelocityParameters(node);

  debug_publisher_ = node.advertise<visualization_msgs::MarkerArray>(ns_ + "/debug_markers", 1);
  virtual_wall_publisher_ = node.advertise<visualization_msgs::MarkerArray>(ns_ + "/virtual_walls", 1);
  debug_trajectory_publisher_ = node.advertise<autoware_planning_msgs::Trajectory>(
    "debug/" + ns_ + "/trajectory", 1);
  processing_diag_publisher_ = std::make_shared<autoware_utils::ProcessingTimePublisher>(
    autoware_utils_debug::ros1::CallerNodeContext::borrow(node_owner_),
    "~/debug/" + ns_ + "/processing_time_ms_diag");

  const auto vehicle_info = vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo();
  vehicle_lateral_offset_ = static_cast<double>(vehicle_info.max_lateral_offset_m);
  vehicle_front_offset_ = static_cast<double>(vehicle_info.max_longitudinal_offset_m);
  distance_buffer_ = autoware_utils::get_or_declare_parameter<double>(node, "distance_buffer");

  projection_params_.wheel_base = vehicle_info.wheel_base_m;
  projection_params_.extra_length = vehicle_front_offset_ + distance_buffer_;
}

void ObstacleVelocityLimiterModule::updateParameters(const ParameterMap & parameters)
{
  using obstacle_velocity_limiter::ObstacleParameters;
  using obstacle_velocity_limiter::PreprocessingParameters;
  using obstacle_velocity_limiter::ProjectionParameters;
  using obstacle_velocity_limiter::VelocityParameters;
  for (const auto & [name, parameter] : parameters) {
    if (name == "distance_buffer") {
      if (!as_double(parameter, distance_buffer_)) { warn_invalid_update(name); continue; }
      projection_params_.extra_length = vehicle_front_offset_ + distance_buffer_;
      // Preprocessing parameters
    } else if (name == PreprocessingParameters::START_DIST_PARAM) {
      if (!as_double(parameter, preprocessing_params_.start_distance)) warn_invalid_update(name);
    } else if (name == PreprocessingParameters::DOWNSAMPLING_PARAM) {
      int value{}; if (!as_int(parameter, value) || !preprocessing_params_.updateDownsampleFactor(value)) warn_invalid_update(name);
    } else if (name == PreprocessingParameters::CALC_STEER_PARAM) {
      if (!as_bool(parameter, preprocessing_params_.calculate_steering_angles)) warn_invalid_update(name);
    } else if (name == PreprocessingParameters::MAX_LENGTH_PARAM) {
      if (!as_double(parameter, preprocessing_params_.max_length)) warn_invalid_update(name);
    } else if (name == PreprocessingParameters::MAX_DURATION_PARAM) {
      if (!as_double(parameter, preprocessing_params_.max_duration)) warn_invalid_update(name);
      // Velocity parameters
    } else if (name == VelocityParameters::MIN_VEL_PARAM) {
      if (!as_double(parameter, velocity_params_.min_velocity)) warn_invalid_update(name);
    } else if (name == VelocityParameters::MAX_DECEL_PARAM) {
      if (!as_double(parameter, velocity_params_.max_deceleration)) warn_invalid_update(name);
      // Obstacle parameters
    } else if (name == ProjectionParameters::DURATION_PARAM) {
      double min_ttc{}; if (!as_double(parameter, min_ttc)) { warn_invalid_update(name); continue; }
      if (min_ttc > 0.0) projection_params_.duration = min_ttc;
      else if (min_ttc <= 0.0) warn_invalid_update(name);
    } else if (name == ObstacleParameters::DYN_SOURCE_PARAM) {
      std::string value; if (!as_string(parameter, value) || !obstacle_params_.updateType(value)) warn_invalid_update(name);
    } else if (name == ObstacleParameters::OCC_GRID_THRESH_PARAM) {
      int value{}; if (as_int(parameter, value)) obstacle_params_.occupancy_grid_threshold = static_cast<int8_t>(value); else warn_invalid_update(name);
    } else if (name == ObstacleParameters::BUFFER_PARAM) {
      if (!as_double(parameter, obstacle_params_.dynamic_obstacles_buffer)) warn_invalid_update(name);
    } else if (name == ObstacleParameters::MIN_VEL_PARAM) {
      if (!as_double(parameter, obstacle_params_.dynamic_obstacles_min_vel)) warn_invalid_update(name);
    } else if (name == ObstacleParameters::MAP_TAGS_PARAM) {
      if (!as_string_array(parameter, obstacle_params_.static_map_tags)) warn_invalid_update(name);
    } else if (name == ObstacleParameters::FILTERING_PARAM) {
      if (!as_bool(parameter, obstacle_params_.filter_envelope)) warn_invalid_update(name);
    } else if (name == ObstacleParameters::IGNORE_ON_PATH_PARAM) {
      if (!as_bool(parameter, obstacle_params_.ignore_on_path)) warn_invalid_update(name);
    } else if (name == ObstacleParameters::IGNORE_DIST_PARAM) {
      if (!as_double(parameter, obstacle_params_.ignore_extra_distance)) warn_invalid_update(name);
    } else if (name == ObstacleParameters::RTREE_POINTS_PARAM) {
      int value{}; if (!as_int(parameter, value) || !obstacle_params_.updateRtreeMinPoints(value)) warn_invalid_update(name);
    } else if (name == ObstacleParameters::RTREE_SEGMENTS_PARAM) {
      int value{}; if (!as_int(parameter, value) || !obstacle_params_.updateRtreeMinSegments(value)) warn_invalid_update(name);
      // Projection parameters
    } else if (name == ProjectionParameters::MODEL_PARAM) {
      std::string value; if (!as_string(parameter, value) || !projection_params_.updateModel(value)) warn_invalid_update(name);
    } else if (name == ProjectionParameters::NB_POINTS_PARAM) {
      int value{}; if (!as_int(parameter, value) || !projection_params_.updateNbPoints(value)) warn_invalid_update(name);
    } else if (name == ProjectionParameters::STEER_OFFSET_PARAM) {
      if (!as_double(parameter, projection_params_.steering_angle_offset)) warn_invalid_update(name);
    } else if (name == ProjectionParameters::DISTANCE_METHOD_PARAM) {
      std::string value; if (!as_string(parameter, value) || !projection_params_.updateDistanceMethod(value)) warn_invalid_update(name);
    } else {
      ROS_WARN("Unknown parameter %s", name.c_str());
    }
  }
}

VelocityPlanningResult ObstacleVelocityLimiterModule::plan(
  [[maybe_unused]] const std::vector<autoware_planning_msgs::TrajectoryPoint> &
    raw_trajectory_points,
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & smoothed_trajectory_points,
  const std::shared_ptr<const PlannerData> & planner_data)
{
  autoware_utils::StopWatch<std::chrono::microseconds> stopwatch;
  stopwatch.tic();
  VelocityPlanningResult result;
  stopwatch.tic("preprocessing");
  const auto ego_idx = autoware::motion_utils::findNearestIndex(
    smoothed_trajectory_points, planner_data->current_odometry.pose.pose);
  if (!ego_idx) {
    ROS_WARN_THROTTLE(1.0, "Cannot calculate ego index on the trajectory");
    return result;
  }
  auto original_traj_points = smoothed_trajectory_points;
  if (preprocessing_params_.calculate_steering_angles)
    obstacle_velocity_limiter::calculateSteeringAngles(
      original_traj_points, projection_params_.wheel_base);
  velocity_params_.current_ego_velocity = planner_data->current_odometry.twist.twist.linear.x;
  const auto start_idx = obstacle_velocity_limiter::calculateStartIndex(
    original_traj_points, *ego_idx, preprocessing_params_.start_distance);
  const auto end_idx = obstacle_velocity_limiter::calculateEndIndex(
    original_traj_points, start_idx, preprocessing_params_.max_length,
    preprocessing_params_.max_duration);
  auto downsampled_traj_points = obstacle_velocity_limiter::downsampleTrajectory(
    original_traj_points, start_idx, end_idx, preprocessing_params_.downsample_factor);
  if (prev_inserted_point_) {
    obstacle_velocity_limiter::add_trajectory_point(downsampled_traj_points, *prev_inserted_point_);
  }
  obstacle_velocity_limiter::ObstacleMasks obstacle_masks;
  const auto preprocessing_us = stopwatch.toc("preprocessing");
  stopwatch.tic("obstacles");
  obstacle_masks.negative_masks = obstacle_velocity_limiter::createPolygonMasks(
    planner_data->objects, obstacle_params_.dynamic_obstacles_buffer,
    obstacle_params_.dynamic_obstacles_min_vel);
  if (obstacle_params_.ignore_on_path)
    obstacle_masks.negative_masks.push_back(
      obstacle_velocity_limiter::createTrajectoryFootprint(
        original_traj_points, vehicle_lateral_offset_ + obstacle_params_.ignore_extra_distance));
  const auto projected_linestrings =
    obstacle_velocity_limiter::createProjectedLines(downsampled_traj_points, projection_params_);
  const auto footprint_polygons = obstacle_velocity_limiter::createFootprintPolygons(
    projected_linestrings, vehicle_lateral_offset_);
  obstacle_velocity_limiter::Obstacles obstacles;
  obstacles.lines = obstacle_velocity_limiter::extractStaticObstacles(
    *planner_data->route_handler->getLaneletMapPtr(), obstacle_params_.static_map_tags,
    footprint_polygons);
  if (
    obstacle_params_.dynamic_source != obstacle_velocity_limiter::ObstacleParameters::STATIC_ONLY) {
    if (obstacle_params_.filter_envelope)
      obstacle_masks.positive_mask =
        obstacle_velocity_limiter::createEnvelopePolygon(footprint_polygons);
    obstacle_velocity_limiter::addSensorObstacles(
      obstacles, planner_data->occupancy_grid_map, planner_data->no_ground_pointcloud.pointcloud,
      obstacle_masks, obstacle_params_);
  }
  const auto obstacles_us = stopwatch.toc("obstacles");
  autoware::motion_utils::VirtualWalls virtual_walls;
  stopwatch.tic("slowdowns");
  result.slowdown_intervals = obstacle_velocity_limiter::calculate_slowdown_intervals(
    downsampled_traj_points,
    obstacle_velocity_limiter::CollisionChecker(
      obstacles, obstacle_params_.rtree_min_points, obstacle_params_.rtree_min_segments),
    projected_linestrings, footprint_polygons, projection_params_, velocity_params_, virtual_walls);
  const auto slowdowns_us = stopwatch.toc("slowdowns");

  for (auto & wall : virtual_walls) {
    wall.longitudinal_offset = vehicle_front_offset_;
    wall.text = ns_;
    wall.style = autoware::motion_utils::VirtualWallType::slowdown;
  }
  virtual_wall_marker_creator_.add_virtual_walls(virtual_walls);
  virtual_wall_publisher_.publish(virtual_wall_marker_creator_.create_markers());
  if (!result.slowdown_intervals.empty()) {
    prev_inserted_point_ = result.slowdown_intervals.front().from;
  }

  const auto total_us = stopwatch.toc();
  ROS_DEBUG_NAMED(
    module_name_.c_str(),
    "Total runtime: %2.2fus\n"
    "\tpreprocessing = %2.0fus\n"
    "\tobstacles = %2.0fus\n"
    "\tslowdowns = %2.0fus\n",
    total_us, preprocessing_us, obstacles_us, slowdowns_us);
  if (debug_publisher_.getNumSubscribers() > 0) {
    const auto safe_projected_linestrings =
      obstacle_velocity_limiter::createProjectedLines(downsampled_traj_points, projection_params_);
    const auto safe_footprint_polygons = obstacle_velocity_limiter::createFootprintPolygons(
      safe_projected_linestrings, vehicle_lateral_offset_);
    debug_publisher_.publish(makeDebugMarkers(
      obstacles, projected_linestrings, safe_projected_linestrings, footprint_polygons,
      safe_footprint_polygons, obstacle_masks,
      planner_data->current_odometry.pose.pose.position.z));
  }
  std::map<std::string, double> processing_times;
  processing_times["preprocessing"] = preprocessing_us / 1000;
  processing_times["obstacles"] = obstacles_us / 1000;
  processing_times["slowdowns"] = slowdowns_us / 1000;
  processing_times["Total"] = total_us / 1000;
  processing_diag_publisher_->publish(processing_times);
  return result;
}
}  // namespace autoware::motion_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::motion_velocity_planner::ObstacleVelocityLimiterModule,
  autoware::motion_velocity_planner::PluginModuleInterface)
