// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#include "obstacle_cruise_module.hpp"
#include "body_longitudinal_filter.hpp"
#include "lateral_collision_check.hpp"
#include "trajectory_index_mapping.hpp"
#include "ros1_compat.hpp"

#include "autoware_utils/ros/uuid_helper.hpp"

#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_utils/ros/parameter.hpp>
#include <autoware_utils/ros/update_param.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <std_msgs/String.h>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner
{
namespace
{
std::vector<PredictedPath> resample_highest_confidence_predicted_paths(
  const std::vector<PredictedPath> & predicted_paths, const double time_interval,
  const double time_horizon, const size_t num_paths)
{
  std::vector<PredictedPath> sorted_paths = predicted_paths;

  // Sort paths by descending confidence
  std::sort(
    sorted_paths.begin(), sorted_paths.end(),
    [](const PredictedPath & a, const PredictedPath & b) { return a.confidence > b.confidence; });

  std::vector<PredictedPath> selected_paths;
  size_t path_count = 0;

  // Select paths that meet the confidence thresholds
  for (const auto & path : sorted_paths) {
    if (path_count < num_paths) {
      selected_paths.push_back(path);
      ++path_count;
    }
  }

  // Resample each selected path
  std::vector<PredictedPath> resampled_paths;
  for (const auto & path : selected_paths) {
    if (path.path.size() < 2) {
      continue;
    }
    resampled_paths.push_back(
      autoware::object_recognition_utils::resamplePredictedPath(path, time_interval, time_horizon));
  }

  return resampled_paths;
}

VelocityLimitClearCommand create_velocity_limit_clear_command(
  const ros::Time & current_time, [[maybe_unused]] const std::string & module_name)
{
  VelocityLimitClearCommand msg;
  msg.stamp.sec = current_time.sec;
  msg.stamp.nanosec = current_time.nsec;
  msg.sender = "obstacle_cruise";
  msg.command = true;
  return msg;
}
}  // namespace

void ObstacleCruiseModule::init(ros::NodeHandle & node, const std::string & module_name)
{
  module_name_ = module_name;

  // ros parameters
  planning_algorithm_ =
    get_or_declare_parameter<std::string>(node, "obstacle_cruise.option.planning_algorithm");
  common_param_ = CruiseCommonParam(node);
  cruise_planning_param_ = CruisePlanningParam(node);
  obstacle_filtering_param_ = CruiseObstacleFilteringParam(node);

  // common publisher
  virtual_wall_publisher_ =
    node.advertise<MarkerArray>("obstacle_cruise/virtual_walls", 1);
  debug_publisher_ = node.advertise<MarkerArray>("obstacle_cruise/debug_markers", 1);

  // module publisher
  debug_cruise_planning_info_pub_ =
    node.advertise<Float32MultiArrayStamped>("debug/obstacle_cruise/planning_info", 1);
  debug_selection_pub_ =
    node.advertise<std_msgs::String>("debug/obstacle_cruise/selection", 1);
  debug_input_trajectory_pub_ = node.advertise<autoware_planning_msgs::Trajectory>(
    "debug/obstacle_cruise/input_trajectory", 1);
  processing_time_detail_pub_ = node.advertise<autoware_utils::ProcessingTimeDetail>(
    "debug/processing_time_detail_ms/obstacle_cruise", 1);

  // interface
  objects_of_interest_marker_interface_ = std::make_unique<
    autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface>(
    &node, "obstacle_cruise");
  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      &node, "obstacle_cruise");

  // time keeper
  time_keeper_ = std::make_shared<autoware_utils::TimeKeeper>(processing_time_detail_pub_);

  // debug publisher
  debug_trajectory_publisher_ = node.advertise<autoware_planning_msgs::Trajectory>(
    "debug/obstacle_cruise/trajectory", 1);

  // cruise planner
  cruise_planner_ = create_cruise_planner(node);
}

void ObstacleCruiseModule::updateParameters(const ParameterMap & parameters)
{
  cruise_planner_->updateParameters(parameters);
}

VelocityPlanningResult ObstacleCruiseModule::plan(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & raw_trajectory_points,
  [[maybe_unused]] const std::vector<autoware_planning_msgs::TrajectoryPoint> &
    smoothed_trajectory_points,
  const std::shared_ptr<const PlannerData> & planner_data)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // 1. init variables
  debug_data_ptr_ = std::make_shared<ObstacleCruiseDebugData>();
  filter_trace_.clear();
  if (debug_input_trajectory_pub_.getNumSubscribers() > 0) {
    autoware_planning_msgs::Trajectory input;
    input.header.frame_id = "map";
    input.header.stamp = ros::Time::now();
    input.points = raw_trajectory_points;
    debug_input_trajectory_pub_.publish(input);
  }

  // filter obstacles of predicted objects
  const lateral_collision::LongitudinalMotion ego_motion{
    planner_data->current_acceleration.accel.accel.linear.x,
    common_param_.min_accel, common_param_.max_accel,
    common_param_.min_jerk, common_param_.max_jerk};
  const auto cruise_obstacles = filter_cruise_obstacle_for_predicted_object(
    planner_data->current_odometry, ego_motion, planner_data->ego_nearest_dist_threshold,
    planner_data->ego_nearest_yaw_threshold, raw_trajectory_points, planner_data->objects,
    ros::Time(planner_data->predicted_objects_header.stamp), planner_data->is_driving_forward,
    planner_data->vehicle_info_, planner_data->trajectory_polygon_collision_check);

  // plan cruise
  VelocityPlanningResult result;
  [[maybe_unused]] const auto cruise_traj_points = cruise_planner_->plan_cruise(
    planner_data, raw_trajectory_points, cruise_obstacles, debug_data_ptr_,
    planning_factor_interface_, result.velocity_limit);

  // Diagnostic only: identify input, accepted candidates and the actual PID target
  // in one cycle. No additional filtering or control-state updates are performed.
  if (debug_selection_pub_.getNumSubscribers() > 0) {
    std::ostringstream stream;
    stream << std::setprecision(17);
    const auto number = [&stream](const double value) {
      if (std::isfinite(value)) stream << value;
      else stream << "null";
    };
    stream << "{\"schema_version\":1,\"stamp\":";
    number(ros::Time::now().toSec());
    stream << ",\"objects_stamp\":";
    number(ros::Time(planner_data->predicted_objects_header.stamp).toSec());
    stream << ",\"ego_velocity_mps\":";
    number(planner_data->current_odometry.twist.twist.linear.x);
    stream << ",\"inputs\":[";
    bool first = true;
    for (const auto & object : planner_data->objects) {
      if (!first) stream << ',';
      first = false;
      const auto & predicted = object->predicted_object;
      const auto & pos = predicted.kinematics.initial_pose_with_covariance.pose.position;
      const auto accepted = std::any_of(
        cruise_obstacles.begin(), cruise_obstacles.end(),
        [&predicted](const auto & candidate) { return candidate.uuid.uuid == predicted.object_id.uuid; });
      const auto selected = std::any_of(
        debug_data_ptr_->obstacles_to_cruise.begin(), debug_data_ptr_->obstacles_to_cruise.end(),
        [&predicted](const auto & candidate) { return candidate.uuid.uuid == predicted.object_id.uuid; });
      stream << "{\"uuid\":\"" << autoware_utils::to_hex_string(predicted.object_id)
             << "\",\"x\":";
      number(pos.x);
      stream << ",\"y\":";
      number(pos.y);
      const auto key = autoware_utils::to_hex_string(predicted.object_id);
      const auto & trace = filter_trace_[key];
      stream << ",\"filter_reason\":\"" << trace.reason << "\",\"filter_values\":{";
      bool first_value = true;
      for (const auto & value : trace.values) {
        if (!first_value) stream << ',';
        first_value = false;
        stream << '\"' << value.first << "\":";
        number(value.second);
      }
      stream << "},\"candidate\":" << (accepted ? "true" : "false")
             << ",\"selected\":" << (selected ? "true" : "false") << '}';
    }
    stream << "],\"candidate_uuids\":[";
    first = true;
    for (const auto & candidate : cruise_obstacles) {
      if (!first) stream << ',';
      first = false;
      stream << '\"' << autoware_utils::to_hex_string(candidate.uuid) << '\"';
    }
    stream << "],\"selected_uuids\":[";
    first = true;
    for (const auto & selected : debug_data_ptr_->obstacles_to_cruise) {
      if (!first) stream << ',';
      first = false;
      stream << '\"' << autoware_utils::to_hex_string(selected.uuid) << '\"';
    }
    stream << "],\"velocity_limit_mps\":";
    if (result.velocity_limit) number(result.velocity_limit->max_velocity);
    else stream << "null";
    stream << ",\"clear_velocity_limit\":"
           << ((!result.velocity_limit && need_to_clear_velocity_limit_) ? "true" : "false") << '}';
    std_msgs::String message;
    message.data = stream.str();
    debug_selection_pub_.publish(message);
  }

  // clear velocity limit if necessary
  if (result.velocity_limit) {
    need_to_clear_velocity_limit_ = true;
  } else {
    if (need_to_clear_velocity_limit_) {
      // clear velocity limit
      result.velocity_limit_clear_command =
        create_velocity_limit_clear_command(ros::Time::now(), module_name_);
      need_to_clear_velocity_limit_ = false;
    }
  }

  publish_debug_info();

  return result;
}

std::string ObstacleCruiseModule::getModuleName() const
{
  return module_name_;
}

std::unique_ptr<CruisePlannerInterface> ObstacleCruiseModule::create_cruise_planner(
  ros::NodeHandle & node) const
{
  if (planning_algorithm_ == "pid_base") {
    return std::make_unique<PIDBasedPlanner>(node, common_param_, cruise_planning_param_);
  } else if (planning_algorithm_ == "optimization_base") {
    return std::make_unique<OptimizationBasedPlanner>(node, common_param_, cruise_planning_param_);
  }
  throw std::logic_error("Designated algorithm is not supported.");
}

std::vector<CruiseObstacle> ObstacleCruiseModule::filter_cruise_obstacle_for_predicted_object(
  const Odometry & odometry, const lateral_collision::LongitudinalMotion & ego_motion,
  const double ego_nearest_dist_threshold,
  const double ego_nearest_yaw_threshold, const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<std::shared_ptr<PlannerData::Object>> & objects,
  const ros::Time & predicted_objects_stamp, const bool is_driving_forward,
  const VehicleInfo & vehicle_info,
  const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto & current_pose = odometry.pose.pose;

  if (traj_points.size() < 2) return {};
  const auto ego_segment_index =
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      traj_points, current_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
  const auto & p = trajectory_polygon_collision_check;
  const auto decimated_traj_points = utils::decimate_trajectory_points_from_ego(
    traj_points, current_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold,
    p.decimate_trajectory_step_length, 0.0);
  const auto decimated_traj_polys = polygon_utils::createOneStepPolygons(
    decimated_traj_points, vehicle_info, current_pose, 0.0, p.enable_to_consider_current_pose,
    p.time_to_convergence);
  debug_data_ptr_->decimated_traj_polys = decimated_traj_polys;

  // cruise
  std::vector<CruiseObstacle> cruise_obstacles;
  for (const auto & object : objects) {
    auto & trace = filter_trace_[autoware_utils::to_hex_string(object->predicted_object.object_id)];
    // 1. rough filtering
    // 1.1. Exclude a rear reference point only after the entire body clears Ego.
    const double lon_dist_from_ego_to_obj =
      object->get_dist_from_ego_longitudinal(traj_points, current_pose.position);
    trace.values["longitudinal_distance_m"] = lon_dist_from_ego_to_obj;
    if (lon_dist_from_ego_to_obj < 0.0) {
      const auto bounds = body_longitudinal_bounds(
        traj_points, current_pose, object->predicted_object, vehicle_info);
      trace.values["body_longitudinal_bounds_valid"] = bounds.has_value();
      if (bounds) {
        trace.values["object_body_max_longitudinal_m"] = bounds->object_max_s;
        trace.values["ego_body_min_longitudinal_m"] = bounds->ego_min_s;
        trace.values["body_longitudinal_clearance_m"] = bounds->ego_min_s - bounds->object_max_s;
        trace.values["body_footprints_overlap"] = bounds->footprints_overlap;
        if (bounds->fully_behind()) {
          trace.reason = "behind_ego_body";
          continue;
        }
      }
    }

    // 1.2. Check if the rough lateral distance is smaller than the threshold.
    const double min_lat_dist_to_traj_poly =
      utils::calc_possible_min_dist_from_obj_to_traj_poly(object, traj_points, vehicle_info);
    trace.values["rough_min_lateral_gap_m"] = min_lat_dist_to_traj_poly;
    if (obstacle_filtering_param_.max_lat_margin < min_lat_dist_to_traj_poly) {
      trace.reason = "rough_lateral_gap";
      continue;
    }

    // 2. precise filtering for cruise
    const auto cruise_obstacle = create_cruise_obstacle(
      odometry, traj_points, ego_segment_index, ego_motion,
      decimated_traj_points, decimated_traj_polys, object,
      predicted_objects_stamp, object->get_dist_to_traj_poly(decimated_traj_polys),
      is_driving_forward, vehicle_info, trajectory_polygon_collision_check);
    if (cruise_obstacle) {
      trace.reason = "accepted_cruise";
      trace.values["collision_point_count"] = cruise_obstacle->collision_points.size();
      cruise_obstacles.push_back(*cruise_obstacle);
      continue;
    }
  }

  // 3. precise filtering for yield cruise
  if (obstacle_filtering_param_.enable_yield) {
    const auto yield_obstacles = find_yield_cruise_obstacles(
      odometry, objects, predicted_objects_stamp, traj_points, vehicle_info);
    if (yield_obstacles) {
      for (const auto & y : yield_obstacles.value()) {
        // Check if there is no member with the same UUID in cruise_obstacles
        auto it = std::find_if(
          cruise_obstacles.begin(), cruise_obstacles.end(),
          [&y](const auto & c) { return y.uuid == c.uuid; });

        // If no matching UUID found, insert yield obstacle into cruise_obstacles
        if (it == cruise_obstacles.end()) {
          filter_trace_[autoware_utils::to_hex_string(y.uuid)].reason = "accepted_yield";
          cruise_obstacles.push_back(y);
        }
      }
    }
  }

  prev_cruise_object_obstacles_ = cruise_obstacles;

  return cruise_obstacles;
}

void ObstacleCruiseModule::publish_debug_info()
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // 1. debug marker
  MarkerArray debug_marker;

  // 1.1. obstacles
  std::vector<geometry_msgs::Point> stop_collision_points;
  for (size_t i = 0; i < debug_data_ptr_->obstacles_to_cruise.size(); ++i) {
    // obstacle
    const auto obstacle_marker = utils::get_object_marker(
      debug_data_ptr_->obstacles_to_cruise.at(i).pose, i, "obstacles", 1.0, 0.6, 0.1);
    debug_marker.markers.push_back(obstacle_marker);

    // collision points
    for (size_t j = 0; j < debug_data_ptr_->obstacles_to_cruise.at(i).collision_points.size();
         ++j) {
      stop_collision_points.push_back(
        debug_data_ptr_->obstacles_to_cruise.at(i).collision_points.at(j).point);
    }
  }

  // 1.2. collision points
  for (size_t i = 0; i < stop_collision_points.size(); ++i) {
    auto collision_point_marker = autoware_utils::create_default_marker(
      "map", ros::Time::now(), "collision_points", i, Marker::SPHERE,
      autoware_utils::create_marker_scale(0.25, 0.25, 0.25),
      autoware_utils::create_marker_color(1.0, 0.0, 0.0, 0.999));
    collision_point_marker.pose.position = stop_collision_points.at(i);
    debug_marker.markers.push_back(collision_point_marker);
  }

  // 1.3. intentionally ignored obstacles
  for (size_t i = 0; i < debug_data_ptr_->intentionally_ignored_obstacles.size(); ++i) {
    const auto marker = utils::get_object_marker(
      debug_data_ptr_->intentionally_ignored_obstacles.at(i)
        ->predicted_object.kinematics.initial_pose_with_covariance.pose,
      i, "intentionally_ignored_obstacles", 0.0, 1.0, 0.0);
    debug_marker.markers.push_back(marker);
  }

  // 1.4. detection area
  auto decimated_traj_polys_marker = autoware_utils::create_default_marker(
    "map", ros::Time::now(), "detection_area", 0, Marker::LINE_LIST,
    autoware_utils::create_marker_scale(0.01, 0.0, 0.0),
    autoware_utils::create_marker_color(0.0, 1.0, 0.0, 0.999));
  for (const auto & decimated_traj_poly : debug_data_ptr_->decimated_traj_polys) {
    for (size_t dp_idx = 0; dp_idx < decimated_traj_poly.outer().size(); ++dp_idx) {
      const auto & current_point = decimated_traj_poly.outer().at(dp_idx);
      const auto & next_point =
        decimated_traj_poly.outer().at((dp_idx + 1) % decimated_traj_poly.outer().size());

      decimated_traj_polys_marker.points.push_back(
        autoware_utils::create_point(current_point.x(), current_point.y(), 0.0));
      decimated_traj_polys_marker.points.push_back(
        autoware_utils::create_point(next_point.x(), next_point.y(), 0.0));
    }
  }
  debug_marker.markers.push_back(decimated_traj_polys_marker);

  debug_publisher_.publish(debug_marker);

  // 2. virtual wall
  virtual_wall_publisher_.publish(debug_data_ptr_->cruise_wall_marker);

  // 3. cruise planning info
  const auto cruise_debug_msg = cruise_planner_->get_cruise_planning_debug_message(ros::Time::now());
  debug_cruise_planning_info_pub_.publish(cruise_debug_msg);

  // 4. objects of interest
  objects_of_interest_marker_interface_->publishMarkerArray();
}

std::optional<CruiseObstacle> ObstacleCruiseModule::create_cruise_obstacle(
  const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
  const size_t ego_segment_index,
  const lateral_collision::LongitudinalMotion & ego_motion,
  const std::vector<TrajectoryPoint> & decimated_traj_points,
  const std::vector<Polygon2d> & decimated_traj_polys,
  const std::shared_ptr<PlannerData::Object> object, const rclcpp::Time & predicted_objects_stamp,
  const double dist_from_obj_poly_to_traj_poly, const bool is_driving_forward,
  const VehicleInfo & vehicle_info,
  const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check) const
{
  const auto & obj_uuid = object->predicted_object.object_id;
  const auto & obj_uuid_str = autoware_utils::to_hex_string(obj_uuid);

  auto & trace = filter_trace_[obj_uuid_str];
  trace.reason = "no_collision_points";
  trace.values["polygon_gap_m"] = dist_from_obj_poly_to_traj_poly;
  trace.values["lateral_velocity_towards_path_mps"] = object->get_lat_vel_relative_to_traj(traj_points);
  trace.values["longitudinal_velocity_mps"] = object->get_lon_vel_relative_to_traj(traj_points);
  // NOTE: When driving backward, Stop will be planned instead of cruise.
  //       When the obstacle is crossing the ego's trajectory, cruise can be ignored.
  if (
    !is_cruise_obstacle(object->predicted_object.classification.at(0).label) ||
    !is_driving_forward) {
    trace.reason = "class_or_reverse";
    return std::nullopt;
  }

  if (object->get_lon_vel_relative_to_traj(traj_points) < 0.0) {
    trace.reason = "opposite_direction";
    RCLCPP_DEBUG(
      logger_, "[Cruise] Ignore obstacle (%s) since it's driving in opposite direction.",
      obj_uuid_str.substr(0, 4).c_str());
    return std::nullopt;
  }

  if (obstacle_filtering_param_.max_lat_margin < dist_from_obj_poly_to_traj_poly) {
    const auto now = ros::Time::now();
    // Use one reference normal for BOTH velocities and the footprint clearance.
    // Object-only motion misses a merge caused by ego's own lateral motion.
    const auto motion = lateral_collision::relative_motion(
      odometry, object->predicted_object,
      object->get_predicted_current_pose(now, predicted_objects_stamp), vehicle_info,
      tf2::getYaw(traj_points.at(ego_segment_index).pose.orientation));
    trace.values["ego_lateral_velocity_mps"] = motion.ego_lateral_velocity_mps;
    trace.values["object_lateral_velocity_mps"] = motion.object_lateral_velocity_mps;
    trace.values["relative_lateral_velocity_mps"] = motion.relative_lateral_velocity_mps;
    trace.values["relative_lateral_closing_velocity_mps"] = motion.closing_velocity_mps;
    trace.values["relative_lateral_clearance_m"] = motion.clearance_m;
    trace.values["relative_lateral_time_sec"] = motion.time_to_contact_sec;

    lateral_collision::PredictionOverlap prediction;
    const auto max_time = obstacle_filtering_param_.max_lateral_time_margin;
    const auto margin = obstacle_filtering_param_.max_lat_margin;
    if (!lateral_collision::passes_time_gate(motion, margin, max_time, prediction)) {
      // A curved/merging plan can collide even with zero instantaneous relative
      // lateral speed. Test future footprints at matching times before rejecting.
      const auto & p = trajectory_polygon_collision_check;
      prediction = lateral_collision::predicted_overlap(
        odometry, traj_points, object->predicted_object, predicted_objects_stamp, now, vehicle_info,
        std::min(max_time, obstacle_filtering_param_.max_prediction_time_for_collision_check),
        obstacle_filtering_param_.num_of_predicted_paths_for_outside_cruise_obstacle,
        ego_segment_index, p.enable_to_consider_current_pose ? p.time_to_convergence : 0.0,
        ego_motion);
      trace.values["time_aligned_prediction_checked"] = 1.0;
      trace.values["time_aligned_collision"] = prediction.collision ? 1.0 : 0.0;
      trace.values["time_aligned_collision_sec"] = prediction.first_time_sec;
      trace.values["time_aligned_constant_velocity_collision"] =
        prediction.constant_velocity_collision ? 1.0 : 0.0;
      trace.values["ego_motion_hypotheses"] = prediction.motion_hypotheses;
      trace.values["collision_target_acceleration_mps2"] =
        prediction.collision_target_acceleration_mps2;
      trace.values["ego_prediction_initial_acceleration_mps2"] =
        ego_motion.initial_acceleration_mps2;
    }
    if (!lateral_collision::passes_time_gate(motion, margin, max_time, prediction)) {
      trace.reason = "lateral_relative_time_gate";
      RCLCPP_DEBUG(
        logger_, "[Cruise] Ignore obstacle (%s): no near lateral contact or timed overlap.",
        obj_uuid_str.substr(0, 4).c_str());
      return std::nullopt;
    }
  }

  if (is_obstacle_crossing(traj_points, object)) {
    trace.reason = "crossing_angle";
    RCLCPP_DEBUG(
      logger_, "[Cruise] Ignore obstacle (%s) since it's crossing the ego's trajectory..",
      obj_uuid_str.substr(0, 4).c_str());
    return std::nullopt;
  }

  const auto collision_points = [&]() -> std::optional<std::vector<polygon_utils::PointWithStamp>> {
    constexpr double epsilon = 1e-6;
    if (dist_from_obj_poly_to_traj_poly < epsilon) {
      trace.values["inside_path"] = 1.0;
      // obstacle is inside the trajectory
      return create_collision_points_for_inside_cruise_obstacle(
        traj_points, decimated_traj_points, decimated_traj_polys, object, predicted_objects_stamp,
        is_driving_forward, vehicle_info, trajectory_polygon_collision_check);
    }
    trace.values["inside_path"] = 0.0;
    // obstacle is outside the trajectory
    // If the ego is stopping, do not plan cruise for outside obstacles. Stop will be planned.
    if (odometry.twist.twist.linear.x < 0.1) {
      trace.reason = "ego_stopped_outside";
      return std::nullopt;
    }
    return create_collision_points_for_outside_cruise_obstacle(
      traj_points, decimated_traj_points, decimated_traj_polys, object, predicted_objects_stamp,
      is_driving_forward, vehicle_info, trajectory_polygon_collision_check);
  }();
  if (!collision_points) {
    return std::nullopt;
  }

  return CruiseObstacle{
    obj_uuid,
    predicted_objects_stamp,
    object->get_predicted_current_pose(ros::Time::now(), predicted_objects_stamp),
    object->get_lon_vel_relative_to_traj(traj_points),
    object->get_lat_vel_relative_to_traj(traj_points),
    *collision_points};
}

std::optional<std::vector<CruiseObstacle>> ObstacleCruiseModule::find_yield_cruise_obstacles(
  const Odometry & odometry, const std::vector<std::shared_ptr<PlannerData::Object>> & objects,
  const rclcpp::Time & predicted_objects_stamp, const std::vector<TrajectoryPoint> & traj_points,
  const VehicleInfo & vehicle_info)
{
  const auto & current_pose = odometry.pose.pose;

  if (objects.empty() || traj_points.empty()) return std::nullopt;

  std::vector<std::shared_ptr<PlannerData::Object>> stopped_objects;
  std::vector<std::shared_ptr<PlannerData::Object>> moving_objects;

  std::for_each(objects.begin(), objects.end(), [&](const auto & o) {
    const bool is_moving =
      std::hypot(
        o->predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x,
        o->predicted_object.kinematics.initial_twist_with_covariance.twist.linear.y) >
      obstacle_filtering_param_.stopped_obstacle_velocity_threshold;
    if (is_moving) {
      const bool is_within_lat_dist_threshold =
        std::abs(o->get_dist_to_traj_lateral(traj_points)) <
        obstacle_filtering_param_.yield_lat_distance_threshold;
      if (is_within_lat_dist_threshold) moving_objects.push_back(o);
      return;
    }
    // lat threshold is larger for stopped obstacles
    const bool is_within_lat_dist_threshold =
      std::abs(o->get_dist_to_traj_lateral(traj_points)) <
      obstacle_filtering_param_.yield_lat_distance_threshold +
        obstacle_filtering_param_.max_lat_dist_between_obstacles;
    if (is_within_lat_dist_threshold) stopped_objects.push_back(o);
    return;
  });

  if (stopped_objects.empty() || moving_objects.empty()) return std::nullopt;

  std::sort(stopped_objects.begin(), stopped_objects.end(), [&](const auto & o1, const auto & o2) {
    return o1->get_dist_from_ego_longitudinal(traj_points, current_pose.position) <
           o2->get_dist_from_ego_longitudinal(traj_points, current_pose.position);
  });

  std::sort(moving_objects.begin(), moving_objects.end(), [&](const auto & o1, const auto & o2) {
    return o1->get_dist_from_ego_longitudinal(traj_points, current_pose.position) <
           o2->get_dist_from_ego_longitudinal(traj_points, current_pose.position);
  });

  std::vector<CruiseObstacle> yield_obstacles;
  for (const auto & moving_object : moving_objects) {
    for (const auto & stopped_object : stopped_objects) {
      const bool is_moving_obs_behind_of_stopped_obs =
        moving_object->get_dist_from_ego_longitudinal(traj_points, current_pose.position) <
        stopped_object->get_dist_from_ego_longitudinal(traj_points, current_pose.position);
      const bool is_moving_obs_ahead_of_ego_front =
        moving_object->get_dist_from_ego_longitudinal(traj_points, current_pose.position) >
        vehicle_info.vehicle_length_m;

      if (!is_moving_obs_ahead_of_ego_front || !is_moving_obs_behind_of_stopped_obs) continue;

      const double lateral_distance_between_obstacles = std::abs(
        moving_object->get_dist_to_traj_lateral(traj_points) -
        stopped_object->get_dist_to_traj_lateral(traj_points));

      const double longitudinal_distance_between_obstacles = std::abs(
        moving_object->get_dist_from_ego_longitudinal(traj_points, current_pose.position) -
        stopped_object->get_dist_from_ego_longitudinal(traj_points, current_pose.position));

      const double moving_object_speed = std::hypot(
        moving_object->predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x,
        moving_object->predicted_object.kinematics.initial_twist_with_covariance.twist.linear.y);

      const bool are_obstacles_aligned = lateral_distance_between_obstacles <
                                         obstacle_filtering_param_.max_lat_dist_between_obstacles;
      const bool obstacles_collide_within_threshold_time =
        longitudinal_distance_between_obstacles / moving_object_speed <
        obstacle_filtering_param_.max_obstacles_collision_time;
      if (are_obstacles_aligned && obstacles_collide_within_threshold_time) {
        const auto yield_obstacle = create_yield_cruise_obstacle(
          moving_object, stopped_object, predicted_objects_stamp, traj_points);
        if (yield_obstacle) {
          yield_obstacles.push_back(*yield_obstacle);
          using autoware::objects_of_interest_marker_interface::ColorName;
          objects_of_interest_marker_interface_->insertObjectData(
            stopped_object->predicted_object.kinematics.initial_pose_with_covariance.pose,
            stopped_object->predicted_object.shape, ColorName::RED);
        }
      }
    }
  }
  if (yield_obstacles.empty()) return std::nullopt;
  return yield_obstacles;
}

std::optional<std::vector<polygon_utils::PointWithStamp>>
ObstacleCruiseModule::create_collision_points_for_inside_cruise_obstacle(
  const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<TrajectoryPoint> & decimated_traj_points,
  const std::vector<Polygon2d> & decimated_traj_polys,
  const std::shared_ptr<PlannerData::Object> object, const rclcpp::Time & predicted_objects_stamp,
  const bool is_driving_forward, const VehicleInfo & vehicle_info,
  const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check) const
{
  const auto & obj_uuid = object->predicted_object.object_id;
  const auto & obj_uuid_str = autoware_utils::to_hex_string(obj_uuid);

  auto & trace = filter_trace_[obj_uuid_str];
  // check label
  if (!is_inside_cruise_obstacle(object->predicted_object.classification.at(0).label)) {
    trace.reason = "inside_class";
    RCLCPP_DEBUG(
      logger_, "[Cruise] Ignore inside obstacle (%s) since its type is not designated.",
      obj_uuid_str.substr(0, 4).c_str());
    return std::nullopt;
  }

  {  // consider hysteresis
    // const bool is_prev_obstacle_stop = get_obstacle_from_uuid(prev_stop_obstacles_,
    // obstacle.uuid).has_value();
    const bool is_prev_obstacle_cruise =
      utils::get_obstacle_from_uuid(prev_cruise_object_obstacles_, obj_uuid).has_value();

    if (is_prev_obstacle_cruise) {
      if (
        object->get_lon_vel_relative_to_traj(traj_points) <
        obstacle_filtering_param_.obstacle_velocity_threshold_from_cruise) {
        trace.reason = "inside_velocity_hysteresis";
        return std::nullopt;
      }
      // NOTE: else is keeping cruise
    } else {
      // If previous obstacle is stop or does not exist.
      if (
        object->get_lon_vel_relative_to_traj(traj_points) <
        obstacle_filtering_param_.obstacle_velocity_threshold_to_cruise) {
        trace.reason = "inside_velocity_hysteresis";
        return std::nullopt;
      }
      // NOTE: else is cruise from stop
    }
  }

  // Get highest confidence predicted path
  constexpr double prediction_resampling_time_interval = 0.1;
  constexpr double prediction_resampling_time_horizon = 10.0;
  std::vector<PredictedPath> predicted_paths;
  for (const auto & path : object->predicted_object.kinematics.predicted_paths) {
    predicted_paths.push_back(path);
  }
  const auto resampled_predicted_paths = resample_highest_confidence_predicted_paths(
    predicted_paths, prediction_resampling_time_interval, prediction_resampling_time_horizon, 1);

  if (resampled_predicted_paths.empty()) {
    trace.reason = "inside_no_prediction";
    return std::nullopt;
  }

  // calculate nearest collision point
  std::vector<size_t> collision_index;
  const auto collision_points = polygon_utils::getCollisionPoints(
    decimated_traj_points, decimated_traj_polys, predicted_objects_stamp,
    resampled_predicted_paths.front(), object->predicted_object.shape, ros::Time::now(),
    is_driving_forward, collision_index,
    utils::calc_object_possible_max_dist_from_center(object->predicted_object.shape) +
      trajectory_polygon_collision_check.decimate_trajectory_step_length +
      std::hypot(
        vehicle_info.vehicle_length_m,
        vehicle_info.vehicle_width_m * 0.5 + obstacle_filtering_param_.max_lat_margin));
  return collision_points;
}

std::optional<std::vector<polygon_utils::PointWithStamp>>
ObstacleCruiseModule::create_collision_points_for_outside_cruise_obstacle(
  const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<TrajectoryPoint> & decimated_traj_points,
  const std::vector<Polygon2d> & decimated_traj_polys,
  const std::shared_ptr<PlannerData::Object> object, const rclcpp::Time & predicted_objects_stamp,
  const bool is_driving_forward, const VehicleInfo & vehicle_info,
  const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check) const
{
  const auto & obj_uuid = object->predicted_object.object_id;
  const auto & obj_uuid_str = autoware_utils::to_hex_string(obj_uuid);

  auto & trace = filter_trace_[obj_uuid_str];
  // check label
  if (!is_outside_cruise_obstacle(object->predicted_object.classification.at(0).label)) {
    trace.reason = "outside_class";
    RCLCPP_DEBUG(
      logger_, "[Cruise] Ignore outside obstacle (%s) since its type is not designated.",
      obj_uuid_str.substr(0, 4).c_str());
    return std::nullopt;
  }

  if (
    std::hypot(
      object->predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x,
      object->predicted_object.kinematics.initial_twist_with_covariance.twist.linear.y) <
    obstacle_filtering_param_.outside_obstacle_velocity_threshold) {
    trace.reason = "outside_low_velocity";
    RCLCPP_DEBUG(
      logger_, "[Cruise] Ignore outside obstacle (%s) since the obstacle velocity is low.",
      obj_uuid_str.substr(0, 4).c_str());
    return std::nullopt;
  }

  // Get the highest confidence predicted paths
  constexpr double prediction_resampling_time_interval = 0.1;
  constexpr double prediction_resampling_time_horizon = 10.0;
  std::vector<PredictedPath> predicted_paths;
  for (const auto & path : object->predicted_object.kinematics.predicted_paths) {
    predicted_paths.push_back(path);
  }

  const auto resampled_predicted_paths = resample_highest_confidence_predicted_paths(
    predicted_paths, prediction_resampling_time_interval, prediction_resampling_time_horizon,
    obstacle_filtering_param_.num_of_predicted_paths_for_outside_cruise_obstacle);

  // calculate collision condition for cruise
  std::vector<size_t> collision_index;
  const auto get_collision_points = [&]() -> std::vector<polygon_utils::PointWithStamp> {
    for (const auto & predicted_path : resampled_predicted_paths) {
      const auto collision_points = polygon_utils::getCollisionPoints(
        decimated_traj_points, decimated_traj_polys, predicted_objects_stamp, predicted_path,
        object->predicted_object.shape, ros::Time::now(), is_driving_forward, collision_index,
        utils::calc_object_possible_max_dist_from_center(object->predicted_object.shape) +
          trajectory_polygon_collision_check.decimate_trajectory_step_length +
          std::hypot(
            vehicle_info.vehicle_length_m,
            vehicle_info.vehicle_width_m * 0.5 + obstacle_filtering_param_.max_lat_margin),
        obstacle_filtering_param_.max_prediction_time_for_collision_check);
      if (!collision_points.empty()) {
        return collision_points;
      }
    }
    return {};
  };

  const auto collision_points = get_collision_points();

  trace.values["outside_collision_point_count"] = collision_points.size();
  if (collision_points.empty()) {
    trace.reason = "outside_no_collision_points";
    // Ignore vehicle obstacles outside the trajectory without collision
    RCLCPP_DEBUG(
      logger_, "[Cruise] Ignore outside obstacle (%s) since there are no collision points.",
      obj_uuid_str.substr(0, 4).c_str());
    debug_data_ptr_->intentionally_ignored_obstacles.push_back(object);
    return std::nullopt;
  }

  const double overlap_time =
    (collision_points.back().stamp - collision_points.front().stamp).toSec();
  trace.values["overlap_time_sec"] = overlap_time;
  if (overlap_time < obstacle_filtering_param_.ego_obstacle_overlap_time_threshold) {
    trace.reason = "outside_short_overlap";
    // Ignore vehicle obstacles outside the trajectory, whose predicted path
    // overlaps the ego trajectory in a certain time.
    RCLCPP_DEBUG(
      logger_, "[Cruise] Ignore outside obstacle (%s) since it will not collide with the ego.",
      obj_uuid_str.substr(0, 4).c_str());
    debug_data_ptr_->intentionally_ignored_obstacles.push_back(object);
    return std::nullopt;
  }

  // Ignore obstacles behind the ego vehicle.
  // Note: Only using isFrontObstacle(), behind obstacles cannot be filtered
  // properly when the trajectory is crossing or overlapping.
  const size_t first_collision_index = collision_index.front();
  const auto first_collision_index_in_raw = map_collision_index_to_original(
    traj_points, decimated_traj_points, first_collision_index);
  if (!first_collision_index_in_raw) {
    trace.reason = "collision_index_mapping_failed";
    RCLCPP_WARN(logger_, "[Cruise] Could not map collision sample to original trajectory.");
    return std::nullopt;
  }

  const auto raw_object_index = autoware::motion_utils::findNearestIndex(
    traj_points, object->predicted_object.kinematics.initial_pose_with_covariance.pose.position);
  const auto decimated_object_index = autoware::motion_utils::findNearestIndex(
    decimated_traj_points, object->predicted_object.kinematics.initial_pose_with_covariance.pose.position);
  trace.values["collision_index_in_decimated"] = first_collision_index;
  trace.values["collision_index_in_raw"] = *first_collision_index_in_raw;
  trace.values["object_index_in_raw"] = raw_object_index;
  trace.values["object_index_in_decimated"] = decimated_object_index;
  trace.values["raw_point_count"] = traj_points.size();
  trace.values["decimated_point_count"] = decimated_traj_points.size();
  // Keep the pre-fix calculation as evidence only, with an explicit bounds check.
  if (first_collision_index < traj_points.size()) {
    trace.values["front_distance_before_mapping_m"] = autoware::motion_utils::calcSignedArcLength(
      traj_points, raw_object_index, first_collision_index);
  }
  trace.values["front_distance_using_raw_m"] = autoware::motion_utils::calcSignedArcLength(
    traj_points, raw_object_index, *first_collision_index_in_raw);
  trace.values["front_distance_using_decimated_m"] = autoware::motion_utils::calcSignedArcLength(
    decimated_traj_points, decimated_object_index, first_collision_index);
  trace.values["front_pass_using_decimated"] =
    is_front_collide_obstacle(decimated_traj_points, object, first_collision_index) ? 1.0 : 0.0;
  trace.values["collision_index_mapping_error_m"] = autoware_utils::calc_distance2d(
    traj_points.at(*first_collision_index_in_raw).pose.position,
    decimated_traj_points.at(first_collision_index).pose.position);
  trace.values["object_radius_m"] = utils::calc_object_possible_max_dist_from_center(
    object->predicted_object.shape);
  if (!is_front_collide_obstacle(traj_points, object, *first_collision_index_in_raw)) {
    trace.reason = "outside_collision_behind";
    return std::nullopt;
  }
  return collision_points;
}

std::optional<CruiseObstacle> ObstacleCruiseModule::create_yield_cruise_obstacle(
  const std::shared_ptr<PlannerData::Object> object,
  const std::shared_ptr<PlannerData::Object> stopped_object,
  const rclcpp::Time & predicted_objects_stamp, const std::vector<TrajectoryPoint> & traj_points)
{
  if (traj_points.empty()) return std::nullopt;
  // check label

  const auto & obj_uuid = object->predicted_object.object_id;
  const auto & obj_uuid_str = autoware_utils::to_hex_string(obj_uuid);

  if (!is_outside_cruise_obstacle(object->predicted_object.classification.at(0).label)) {
    RCLCPP_DEBUG(
      logger_, "[Cruise] Ignore yield obstacle (%s) since its type is not designated.",
      obj_uuid_str.substr(0, 4).c_str());
    return std::nullopt;
  }

  if (!is_side_stopped_obstacle(stopped_object->predicted_object.classification.at(0).label)) {
    RCLCPP_DEBUG(
      logger_,
      "[Cruise] Ignore yield obstacle (%s) since the corresponding stopped object type is not "
      "designated as side_stopped.",
      obj_uuid_str.substr(0, 4).c_str());
    return std::nullopt;
  }

  if (
    std::hypot(
      object->predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x,
      object->predicted_object.kinematics.initial_twist_with_covariance.twist.linear.y) <
    obstacle_filtering_param_.outside_obstacle_velocity_threshold) {
    RCLCPP_DEBUG(
      logger_, "[Cruise] Ignore yield obstacle (%s) since the obstacle velocity is low.",
      obj_uuid_str.substr(0, 4).c_str());
    return std::nullopt;
  }

  if (is_obstacle_crossing(traj_points, object)) {
    RCLCPP_DEBUG(
      logger_, "[Cruise] Ignore yield obstacle (%s) since it's crossing the ego's trajectory..",
      obj_uuid_str.substr(0, 4).c_str());
    return std::nullopt;
  }

  const auto collision_points = [&]() -> std::optional<std::vector<polygon_utils::PointWithStamp>> {
    const auto obstacle_idx = autoware::motion_utils::findNearestIndex(
      traj_points, object->predicted_object.kinematics.initial_pose_with_covariance.pose);
    if (!obstacle_idx) return std::nullopt;
    const auto collision_traj_point = traj_points.at(obstacle_idx.value());
    const auto & offset = traj_points.at(obstacle_idx.value()).time_from_start;
    const auto object_time = ros::Time::now() + ros::Duration(offset.sec, offset.nanosec);

    polygon_utils::PointWithStamp collision_traj_point_with_stamp;
    collision_traj_point_with_stamp.stamp = object_time;
    collision_traj_point_with_stamp.point.x = collision_traj_point.pose.position.x;
    collision_traj_point_with_stamp.point.y = collision_traj_point.pose.position.y;
    collision_traj_point_with_stamp.point.z = collision_traj_point.pose.position.z;
    std::vector<polygon_utils::PointWithStamp> collision_points_vector{
      collision_traj_point_with_stamp};
    return collision_points_vector;
  }();

  if (!collision_points) return std::nullopt;
  // check if obstacle is driving on the opposite direction
  if (object->get_lon_vel_relative_to_traj(traj_points) < 0.0) return std::nullopt;
  return CruiseObstacle{
    obj_uuid,
    predicted_objects_stamp,
    object->get_predicted_current_pose(ros::Time::now(), predicted_objects_stamp),
    object->get_lon_vel_relative_to_traj(traj_points),
    object->get_lat_vel_relative_to_traj(traj_points),
    collision_points.value(),
    true};
}

bool ObstacleCruiseModule::is_inside_cruise_obstacle(const uint8_t label) const
{
  const auto & types = obstacle_filtering_param_.inside_object_types;
  return std::find(types.begin(), types.end(), label) != types.end();
}

bool ObstacleCruiseModule::is_outside_cruise_obstacle(const uint8_t label) const
{
  const auto & types = obstacle_filtering_param_.outside_object_types;
  return std::find(types.begin(), types.end(), label) != types.end();
}

bool ObstacleCruiseModule::is_side_stopped_obstacle(const uint8_t label) const
{
  const auto & types = obstacle_filtering_param_.side_stopped_object_types;
  return std::find(types.begin(), types.end(), label) != types.end();
}

bool ObstacleCruiseModule::is_cruise_obstacle(const uint8_t label) const
{
  return is_inside_cruise_obstacle(label) || is_outside_cruise_obstacle(label);
}

bool ObstacleCruiseModule::is_front_collide_obstacle(
  const std::vector<TrajectoryPoint> & traj_points,
  const std::shared_ptr<PlannerData::Object> object, const size_t first_collision_idx) const
{
  const auto obstacle_idx = autoware::motion_utils::findNearestIndex(
    traj_points, object->predicted_object.kinematics.initial_pose_with_covariance.pose.position);

  const double obstacle_to_col_points_distance =
    autoware::motion_utils::calcSignedArcLength(traj_points, obstacle_idx, first_collision_idx);
  const double object_possible_max_dist_from_center =
    utils::calc_object_possible_max_dist_from_center(object->predicted_object.shape);

  // If the obstacle is far in front of the collision point, the obstacle is behind the ego.
  return obstacle_to_col_points_distance > -object_possible_max_dist_from_center;
}

bool ObstacleCruiseModule::is_obstacle_crossing(
  const std::vector<TrajectoryPoint> & traj_points,
  const std::shared_ptr<PlannerData::Object> object) const
{
  const double diff_angle = autoware::motion_utils::calc_diff_angle_against_trajectory(
    traj_points, object->predicted_object.kinematics.initial_pose_with_covariance.pose);

  // NOTE: Currently predicted objects does not have orientation availability even
  // though sometimes orientation is not available.
  const bool is_obstacle_crossing_trajectory =
    obstacle_filtering_param_.crossing_obstacle_traj_angle_threshold < std::abs(diff_angle) &&
    obstacle_filtering_param_.crossing_obstacle_traj_angle_threshold < M_PI - std::abs(diff_angle);
  if (!is_obstacle_crossing_trajectory) {
    return false;
  }

  // Only obstacles crossing the ego's trajectory with high speed are considered.
  return true;
}
}  // namespace autoware::motion_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::motion_velocity_planner::ObstacleCruiseModule,
  autoware::motion_velocity_planner::PluginModuleInterface)
