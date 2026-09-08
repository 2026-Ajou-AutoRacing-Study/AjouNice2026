#include "autoware/planning_validator_intersection_collision_checker/intersection_collision_checker_plugin.hpp"

#include <autoware/planning_validator_intersection_collision_checker/lanelet_selection_core_ros1.hpp>
#include <autoware/planning_validator_intersection_collision_checker/ego_trajectory_core_ros1.hpp>
#include <autoware/planning_validator_intersection_collision_checker/left_active_target_core_ros1.hpp>
#include <autoware/planning_validator_intersection_collision_checker/pointcloud_preprocess_core_ros1.hpp>
#include <autoware/planning_validator_intersection_collision_checker/right_active_target_core_ros1.hpp>
#include <autoware/planning_validator_intersection_collision_checker/traffic_regulatory_priority_core_ros1.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <pluginlib/class_list_macros.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>

namespace autoware::planning_validator
{
void IntersectionCollisionCheckerRos1::initialize(ros::NodeHandle & private_nh, const std::string & name)
{
  module_name_ = name;
  std::string error;
  parameters_ready_ = intersection_collision_checker_ros1::load(private_nh, params_, error);
  if (!parameters_ready_) ROS_ERROR_STREAM("ICC ROS1 candidate disabled: " << error);
}

void IntersectionCollisionCheckerRos1::validate(
  const TrajectoryCheckerInput & input, autoware_planning_validator::PlanningValidatorStatus & status)
{
  if (parameters_ready_ && !params_.enable) {
    // Matches the source validate() early return: a deliberately disabled
    // checker does not invalidate the planning trajectory.
    status.is_valid_intersection_collision_check = true;
    return;
  }
  const bool complete_source_inputs = parameters_ready_ && input.current_kinematics &&
    input.obstacle_pointcloud && input.current_acceleration && input.route && input.lanelet_map &&
    input.traffic_signals && input.resampled_trajectory && input.vehicle_info && input.tf_buffer &&
    input.route_handler && input.route_handler->isHandlerReady();
  if (!complete_source_inputs) {
    status.is_valid_intersection_collision_check = false;
    return;
  }
  const auto ego_trajectory =
    intersection_collision_checker::make_ego_trajectory(
      *input.resampled_trajectory, input.current_kinematics->pose.pose, *input.vehicle_info);
  if (!ego_trajectory.has_value()) {
    status.is_valid_intersection_collision_check = false;
    return;
  }
  autoware_planning_msgs::Trajectory front_trajectory;
  front_trajectory.points = ego_trajectory->front_trajectory;
  const auto ego_lanes =
    intersection_collision_checker::select_trajectory_lanelets(
      front_trajectory, *input.route_handler, input.current_kinematics->pose.pose);
  if (!ego_lanes.has_value()) {
    status.is_valid_intersection_collision_check = false;
    return;
  }
  // Source is safe when there is no enabled left/right turn.  This branch is
  // now executable through the same route-bound selection core; active turns
  // remain fail-closed until active targets and tracking are wired together.
  if (ego_lanes->turn_lanelets.empty()) {
    status.is_valid_intersection_collision_check = true;
    return;
  }
  const auto & first_turn = ego_lanes->turn_lanelets.front();
  const auto direction =
    first_turn.hasAttribute("turn_direction") ? first_turn.attribute("turn_direction").value() : "";
  if ((direction == "right" && !params_.right_turn.enable) ||
      (direction == "left" && !params_.left_turn.enable) ||
      (direction != "right" && direction != "left")) {
    status.is_valid_intersection_collision_check = true;
    return;
  }
  const auto now = ros::Time::now();
  const bool right_turn = direction == "right";
  const bool prioritized =
    intersection_collision_checker::ego_turn_has_fresh_signal_priority(
      first_turn, input.traffic_signals, now, 1.0, right_turn);
  const double ego_velocity = input.current_kinematics->twist.twist.linear.x;
  const double time_horizon = std::max(
    params_.min_time_horizon, std::abs(ego_velocity / params_.ego_deceleration));
  const auto active_targets = right_turn
    ? intersection_collision_checker::make_right_active_targets(
        *input.route_handler, front_trajectory, *ego_lanes,
        ego_trajectory->front_index, time_horizon, params_, prioritized)
    : intersection_collision_checker::make_left_active_targets(
        *input.route_handler, front_trajectory, *ego_lanes,
        ego_trajectory->front_index, time_horizon, params_, prioritized);
  if (active_targets.empty()) {
    status.is_valid_intersection_collision_check = true;
    return;
  }
  const intersection_collision_checker_ros1::PointCloudPreprocessParameters cloud_params{
    params_.detection_range, params_.pointcloud.min_height, params_.pointcloud.height_buffer,
    params_.pointcloud.voxel_grid_filter.x, params_.pointcloud.voxel_grid_filter.y,
    params_.pointcloud.voxel_grid_filter.z, params_.pointcloud.voxel_grid_filter.min_size};
  const auto filtered = intersection_collision_checker_ros1::preprocess_pointcloud(
    *input.obstacle_pointcloud, *input.tf_buffer, input.vehicle_info->vehicle_height_m, cloud_params);
  if (!filtered.has_value()) {
    status.is_valid_intersection_collision_check = false;
    return;
  }
  pcl::PointCloud<pcl::PointXYZ> points;
  pcl::fromROSMsg(filtered.value(), points);
  const auto collision = collision_core_.evaluate(
    points, active_targets, now, input.current_kinematics->pose.pose.position.z, ego_velocity, params_);
  const bool consistent_collision = std::any_of(
    collision.collision_lane_ids.begin(), collision.collision_lane_ids.end(),
    [&](const auto id) {
      return std::find(previous_collision_lanes_.begin(), previous_collision_lanes_.end(), id) !=
        previous_collision_lanes_.end();
    });
  previous_collision_lanes_ = collision.collision_lane_ids;
  if (collision.safe) {
    if ((now - last_invalid_time_).toSec() > params_.off_time_buffer) {
      last_valid_time_ = now;
      status.is_valid_intersection_collision_check = true;
      return;
    }
    status.is_valid_intersection_collision_check = false;
    return;
  }
  if ((now - last_valid_time_).toSec() < params_.on_time_buffer || !consistent_collision) {
    status.is_valid_intersection_collision_check = true;
    return;
  }
  last_invalid_time_ = now;
  status.is_valid_intersection_collision_check = false;
}
}  // namespace autoware::planning_validator

PLUGINLIB_EXPORT_CLASS(
  autoware::planning_validator::IntersectionCollisionCheckerRos1,
  autoware::planning_validator::PluginInterface)
