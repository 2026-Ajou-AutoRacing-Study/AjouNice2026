#include "autoware/planning_validator_rear_collision_checker/rear_collision_checker_plugin.hpp"

#include <autoware/planning_validator/rear_pointcloud_tf_core.hpp>
#include <autoware/planning_validator/rear_current_lane_core.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/LaneletSequence.h>
#include <pluginlib/class_list_macros.h>

#include <cmath>

namespace autoware::planning_validator
{
void RearCollisionCheckerRos1::initialize(ros::NodeHandle & private_nh, const std::string & name)
{
  module_name_ = name;
  // Frozen rear_collision_checker.param.yaml defaults for the structs whose
  // neutral core types intentionally have zero initial values.
  blind_spot_range_.range_buffer = 1.0;
  blind_spot_range_.ego_max_deceleration = -4.0;
  blind_spot_range_.ego_min_velocity = 1.38;
  blind_spot_range_.participant_reaction_time = 1.2;
  blind_spot_range_.participant_max_deceleration = -2.0;
  blind_spot_range_.participant_max_velocity = 5.5;
  blind_spot_range_.ttc_margin = 2.0;
  tracking_config_.latency = 0.3;
  tracking_config_.observation_time = 0.3;
  tracking_config_.max_acceleration = 10.0;
  tracking_config_.min_velocity = 1.0;
  blind_spot_safety_.moving_time_threshold = 0.5;
  adjacent_lane_range_.range_buffer = 1.0;
  adjacent_lane_range_.ego_max_deceleration = -4.0;
  adjacent_lane_range_.ego_min_velocity = 1.38;
  adjacent_lane_range_.participant_reaction_time = 1.2;
  adjacent_lane_range_.participant_max_deceleration = -2.0;
  adjacent_lane_range_.participant_max_velocity = 16.6;
  adjacent_lane_safety_.moving_time_threshold = 0.5;
  private_nh.param(
    "common/pointcloud/crop_box_filter/x/min", pointcloud_parameters_.crop_x_min,
    pointcloud_parameters_.crop_x_min);
  private_nh.param(
    "common/pointcloud/crop_box_filter/x/max", pointcloud_parameters_.crop_x_max,
    pointcloud_parameters_.crop_x_max);
  private_nh.param(
    "common/pointcloud/crop_box_filter/z/min", pointcloud_parameters_.crop_z_min,
    pointcloud_parameters_.crop_z_min);
  private_nh.param(
    "common/pointcloud/crop_box_filter/z/max", pointcloud_parameters_.crop_z_max_offset,
    pointcloud_parameters_.crop_z_max_offset);
  private_nh.param(
    "common/pointcloud/voxel_grid_filter/x", pointcloud_parameters_.voxel_x,
    pointcloud_parameters_.voxel_x);
  private_nh.param(
    "common/pointcloud/voxel_grid_filter/y", pointcloud_parameters_.voxel_y,
    pointcloud_parameters_.voxel_y);
  private_nh.param(
    "common/pointcloud/voxel_grid_filter/z", pointcloud_parameters_.voxel_z,
    pointcloud_parameters_.voxel_z);
  private_nh.param(
    "common/pointcloud/clustering/cluster_tolerance", pointcloud_parameters_.cluster_tolerance,
    pointcloud_parameters_.cluster_tolerance);
  private_nh.param(
    "common/pointcloud/clustering/min_cluster_height", pointcloud_parameters_.cluster_min_height,
    pointcloud_parameters_.cluster_min_height);
  private_nh.param(
    "common/pointcloud/clustering/min_cluster_size", pointcloud_parameters_.cluster_min_size,
    pointcloud_parameters_.cluster_min_size);
  private_nh.param(
    "common/pointcloud/clustering/max_cluster_size", pointcloud_parameters_.cluster_max_size,
    pointcloud_parameters_.cluster_max_size);
  private_nh.param("common/blind_spot/lookahead_time", motion_envelope_constraints_.lookahead_time,
    motion_envelope_constraints_.lookahead_time);
  private_nh.param("common/ego/max_deceleration", motion_envelope_constraints_.max_deceleration,
    motion_envelope_constraints_.max_deceleration);
  private_nh.param("common/ego/max_positive_jerk", motion_envelope_constraints_.max_positive_jerk,
    motion_envelope_constraints_.max_positive_jerk);
  private_nh.param("common/ego/max_negative_jerk", motion_envelope_constraints_.max_negative_jerk,
    motion_envelope_constraints_.max_negative_jerk);
  private_nh.param("common/check_on_unstoppable", turn_activation_config_.check_on_unstoppable,
    turn_activation_config_.check_on_unstoppable);
  shift_activation_config_.check_on_unstoppable = turn_activation_config_.check_on_unstoppable;
  private_nh.param("common/blind_spot/check/left", turn_activation_config_.check_left,
    turn_activation_config_.check_left);
  private_nh.param("common/blind_spot/check/right", turn_activation_config_.check_right,
    turn_activation_config_.check_right);
  private_nh.param("common/blind_spot/check/yaw_th", turn_activation_config_.yaw_threshold,
    turn_activation_config_.yaw_threshold);
  private_nh.param("common/pointcloud/range/buffer", blind_spot_range_.range_buffer,
    blind_spot_range_.range_buffer);
  private_nh.param("common/ego/max_deceleration", blind_spot_range_.ego_max_deceleration,
    blind_spot_range_.ego_max_deceleration);
  private_nh.param("common/ego/min_velocity", blind_spot_range_.ego_min_velocity,
    blind_spot_range_.ego_min_velocity);
  private_nh.param("common/time_to_collision/margin", blind_spot_range_.ttc_margin,
    blind_spot_range_.ttc_margin);
  private_nh.param("common/blind_spot/participants/reaction_time",
    blind_spot_range_.participant_reaction_time, blind_spot_range_.participant_reaction_time);
  private_nh.param("common/blind_spot/participants/max_deceleration",
    blind_spot_range_.participant_max_deceleration, blind_spot_range_.participant_max_deceleration);
  private_nh.param("common/blind_spot/participants/max_velocity",
    blind_spot_range_.participant_max_velocity, blind_spot_range_.participant_max_velocity);
  private_nh.param("common/pointcloud/latency", tracking_config_.latency, tracking_config_.latency);
  private_nh.param("common/pointcloud/velocity_estimation/observation_time",
    tracking_config_.observation_time, tracking_config_.observation_time);
  private_nh.param("common/pointcloud/velocity_estimation/max_acceleration",
    tracking_config_.max_acceleration, tracking_config_.max_acceleration);
  private_nh.param("common/filter/min_velocity", tracking_config_.min_velocity,
    tracking_config_.min_velocity);
  private_nh.param("common/filter/moving_time", blind_spot_safety_.moving_time_threshold,
    blind_spot_safety_.moving_time_threshold);
  private_nh.param("common/pointcloud/range/dead_zone", pointcloud_dead_zone_, pointcloud_dead_zone_);
  private_nh.param("common/on_time_buffer", on_time_buffer_, on_time_buffer_);
  private_nh.param("common/off_time_buffer", off_time_buffer_, off_time_buffer_);
  blind_spot_safety_.participant_reaction_time = blind_spot_range_.participant_reaction_time;
  blind_spot_safety_.participant_max_deceleration = blind_spot_range_.participant_max_deceleration;
  blind_spot_safety_.participant_max_velocity = blind_spot_range_.participant_max_velocity;
  blind_spot_safety_.ego_max_deceleration = blind_spot_range_.ego_max_deceleration;
  blind_spot_safety_.ego_min_velocity = blind_spot_range_.ego_min_velocity;
  blind_spot_safety_.ttc_margin = blind_spot_range_.ttc_margin;
  blind_spot_safety_.min_velocity = tracking_config_.min_velocity;
  private_nh.param("common/pointcloud/range/buffer", adjacent_lane_range_.range_buffer,
    adjacent_lane_range_.range_buffer);
  private_nh.param("common/ego/max_deceleration", adjacent_lane_range_.ego_max_deceleration,
    adjacent_lane_range_.ego_max_deceleration);
  private_nh.param("common/ego/min_velocity", adjacent_lane_range_.ego_min_velocity,
    adjacent_lane_range_.ego_min_velocity);
  private_nh.param("common/adjacent_lane/participants/reaction_time",
    adjacent_lane_range_.participant_reaction_time, adjacent_lane_range_.participant_reaction_time);
  private_nh.param("common/adjacent_lane/participants/max_deceleration",
    adjacent_lane_range_.participant_max_deceleration, adjacent_lane_range_.participant_max_deceleration);
  private_nh.param("common/adjacent_lane/participants/max_velocity",
    adjacent_lane_range_.participant_max_velocity, adjacent_lane_range_.participant_max_velocity);
  private_nh.param("common/filter/moving_time", adjacent_lane_safety_.moving_time_threshold,
    adjacent_lane_safety_.moving_time_threshold);
  adjacent_lane_safety_.participant_reaction_time = adjacent_lane_range_.participant_reaction_time;
  adjacent_lane_safety_.participant_max_deceleration = adjacent_lane_range_.participant_max_deceleration;
  adjacent_lane_safety_.participant_max_velocity = adjacent_lane_range_.participant_max_velocity;
  adjacent_lane_safety_.ego_max_deceleration = adjacent_lane_range_.ego_max_deceleration;
  adjacent_lane_safety_.ego_min_velocity = adjacent_lane_range_.ego_min_velocity;
  adjacent_lane_safety_.min_velocity = tracking_config_.min_velocity;
  tracking_core_ = std::make_unique<RearObjectTrackingCore>(tracking_config_);
}

void RearCollisionCheckerRos1::validate(
  const TrajectoryCheckerInput & input,
  autoware_planning_validator::PlanningValidatorStatus & status)
{
  // All source-required data must be received through the node-owned context.
  // No adapter may fabricate a message, a RouteHandler, or a transform.
  const bool complete_input = input.current_kinematics && input.obstacle_pointcloud &&
    input.current_acceleration && input.route && input.lanelet_map &&
    input.resampled_trajectory && input.vehicle_info && input.tf_buffer && input.route_handler;
  if (!complete_input) {
    status.is_valid_rear_collision_check = false;
    return;
  }
  // A geometric inactive branch is not permission to accept malformed input.
  const auto finite_pose = [](const geometry_msgs::Pose & pose) {
    return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
      std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
      std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
      std::isfinite(pose.orientation.w);
  };
  const auto & points = input.resampled_trajectory->points;
  bool finite_input = points.size() >= 2 && finite_pose(input.current_kinematics->pose.pose) &&
    std::isfinite(input.current_kinematics->twist.twist.linear.x) &&
    std::isfinite(input.current_acceleration->accel.accel.linear.x) &&
    std::isfinite(input.vehicle_info->max_longitudinal_offset_m) &&
    input.vehicle_info->max_longitudinal_offset_m >= 0 &&
    std::isfinite(input.vehicle_info->vehicle_width_m) && input.vehicle_info->vehicle_width_m > 0 &&
    std::isfinite(motion_envelope_constraints_.lookahead_time) &&
    std::isfinite(motion_envelope_constraints_.max_deceleration) &&
    std::isfinite(motion_envelope_constraints_.max_positive_jerk) &&
    std::isfinite(motion_envelope_constraints_.max_negative_jerk);
  for (const auto & point : points) {
    finite_input = finite_input && finite_pose(point.pose) &&
      std::isfinite(point.longitudinal_velocity_mps) && std::isfinite(point.acceleration_mps2);
  }
  if (!finite_input) {
    status.is_valid_rear_collision_check = false;
    return;
  }

  // Render the source activation order using the node-owned RouteHandler. The
  // raw pointer is deliberately not wrapped or reinitialized by this plugin.
  const auto current_lanes = get_rear_current_lanes(
    input.route_handler, input.current_kinematics->pose.pose, 100.0, 100.0);
  if (current_lanes.empty()) {
    status.is_valid_rear_collision_check = false;
    return;
  }
  const auto motion_envelope = compute_rear_motion_envelope(
    {*input.resampled_trajectory, *input.current_kinematics, *input.current_acceleration,
     *input.vehicle_info},
    motion_envelope_constraints_);
  if (!motion_envelope) {
    // Original utils::check_turn_behavior and check_shift_behavior each return
    // NONE when their future/stop line cannot be placed on the finite path.
    // The source then accepts the inactive rear-check branch. In particular,
    // a valid terminal parking path can be shorter than the vehicle-front
    // offset; that is not a missing sensor or an observed rear collision.
    status.is_valid_rear_collision_check = true;
    return;
  }
  const auto turn_activation = evaluate_rear_turn_activation(
    {current_lanes, *input.resampled_trajectory, *input.current_kinematics, *input.vehicle_info,
     *motion_envelope, input.route_handler, false},
    turn_activation_config_);
  const auto shift_activation = evaluate_rear_shift_activation(
    {current_lanes, *input.resampled_trajectory, *input.current_kinematics, *input.vehicle_info,
     *motion_envelope, false},
    shift_activation_config_);
  if (turn_activation.behavior == RearBehavior::NONE &&
    shift_activation.behavior == RearBehavior::NONE) {
    // This is the source's inactive branch. Completeness and lane readiness
    // were already checked above, so a rear check is valid without PCL work.
    status.is_valid_rear_collision_check = true;
    return;
  }
  auto blind_spot_range_input = blind_spot_range_;
  blind_spot_range_input.ego_velocity = input.current_kinematics->twist.twist.linear.x;
  blind_spot_range_input.ego_max_longitudinal_offset = input.vehicle_info->max_longitudinal_offset_m;
  blind_spot_range_input.ego_min_longitudinal_offset = input.vehicle_info->min_longitudinal_offset_m;
  blind_spot_range_input.distance_to_conflict_point = turn_activation.distance_to_conflict;
  const auto blind_spot_range = rear_range_for_ttc(blind_spot_range_input);
  auto adjacent_lane_range_input = adjacent_lane_range_;
  adjacent_lane_range_input.ego_velocity = input.current_kinematics->twist.twist.linear.x;
  adjacent_lane_range_input.ego_max_longitudinal_offset = input.vehicle_info->max_longitudinal_offset_m;
  adjacent_lane_range_input.ego_min_longitudinal_offset = input.vehicle_info->min_longitudinal_offset_m;
  const auto adjacent_lane_range = rear_range_for_rss(adjacent_lane_range_input);

  // First executable source branch rendered in R003: raw PointCloud2 ->
  // map-frame transform -> crop/voxel preprocessing -> clustered hull points.
  // The source's fallback-to-default-transform behavior is intentionally not
  // carried forward: absent TF is an invalid safety check.
  const auto source_to_map = lookup_rear_pointcloud_to_map(
    *input.tf_buffer, input.obstacle_pointcloud->header);
  if (!source_to_map) {
    status.is_valid_rear_collision_check = false;
    return;
  }
  try {
    const auto filtered = preprocess_rear_pointcloud(
      *input.obstacle_pointcloud, pointcloud_parameters_, input.vehicle_info->vehicle_height_m,
      *source_to_map);
    const auto clustered = cluster_rear_pointcloud(
      filtered, pointcloud_parameters_, input.current_kinematics->pose.pose.position.z);
    const auto pointcloud_stamp = input.obstacle_pointcloud->header.stamp.toSec();
    const auto ros_now = ros::Time::now().toSec();
    const auto ego_velocity = input.current_kinematics->twist.twist.linear.x;
    std::vector<RearSafetyObject> safety_objects;
    bool object_identity_failure = false;
    const auto append_safety_object =
      [&](const RearProjectedObject & object, const std::vector<RearDetectionArea> & areas,
        const double ego_to_furthest_point, const RearObjectSafetyInput & safety_template,
        const bool use_ttc) {
        const lanelet::ConstLanelet * furthest_lane = nullptr;
        for (const auto & area : areas) {
          if (!area.lanes.empty() && area.lanes.back().id() == object.furthest_lane_id) {
            furthest_lane = &area.lanes.back();
            break;
          }
        }
        if (!furthest_lane || !tracking_core_) return false;

        RearObjectTrackingState tracked;
        tracked.lane_id = furthest_lane->id();
        tracked.last_update_time = pointcloud_stamp;
        tracked.last_stop_time = ros_now;
        tracked.relative_distance = object.absolute_distance - ego_to_furthest_point -
          std::abs(input.vehicle_info->min_longitudinal_offset_m);
        std::vector<int64_t> previous_lane_ids;
        for (const auto & previous : input.route_handler->getPreviousLanelets(*furthest_lane)) {
          previous_lane_ids.push_back(previous.id());
        }
        tracking_core_->update(tracked, ego_velocity, ros_now, previous_lane_ids);

        auto safety_input = safety_template;
        safety_input.relative_distance_with_delay_compensation =
          tracked.relative_distance_with_delay_compensation;
        safety_input.velocity = tracked.velocity;
        safety_input.moving_time = tracked.moving_time;
        safety_input.ego_velocity = ego_velocity;
        auto safety = use_ttc ? evaluate_rear_ttc_safety(safety_input) :
          evaluate_rear_rss_safety(safety_input);
        safety.tracking_duration = tracked.tracking_duration;
        safety_objects.push_back(safety);
        return true;
      };
    if (shift_activation.behavior != RearBehavior::NONE) {
      const bool is_right = shift_activation.behavior == RearBehavior::SHIFT_RIGHT;
      lanelet::ConstLanelets connected_adjacent_lanes;
      double length = 0.0;
      const auto ego_arc = autoware::experimental::lanelet2_utils::get_arc_coordinates(
        current_lanes, input.current_kinematics->pose.pose).length;
      for (const auto & lane : current_lanes) {
        const auto current_lane_length = lanelet::geometry::length2d(lane);
        length += current_lane_length;
        const auto ego_to_furthest = length - ego_arc;
        const auto residual_distance = ego_to_furthest - adjacent_lane_range.forward;
        const auto adjacent = is_right
          ? input.route_handler->getRightLanelet(lane, true, true)
          : input.route_handler->getLeftLanelet(lane, true, true);
        if (adjacent) connected_adjacent_lanes.push_back(*adjacent);
        if (!connected_adjacent_lanes.empty() && residual_distance > 0.0) {
          auto areas = make_rear_previous_detection_areas(
            current_lanes, connected_adjacent_lanes, residual_distance,
            residual_distance + adjacent_lane_range.forward + adjacent_lane_range.backward,
            input.route_handler, -0.5, -0.5);
          cut_rear_detection_areas_by_current_lanes(current_lanes, areas);
          const auto adjacent_object = assemble_rear_object_from_detection_areas(
            *clustered, areas, input.route_handler);
          if (adjacent_object) {
            const auto relative_distance = adjacent_object->absolute_distance - ego_to_furthest -
              std::abs(input.vehicle_info->min_longitudinal_offset_m);
            if (relative_distance >= pointcloud_dead_zone_ - adjacent_lane_range.forward &&
              !append_safety_object(*adjacent_object, areas, ego_to_furthest,
                adjacent_lane_safety_, false)) {
              object_identity_failure = true;
            }
          }
          break;
        }
        if (!connected_adjacent_lanes.empty() && !adjacent) {
          auto areas = make_rear_previous_detection_areas(
            current_lanes, connected_adjacent_lanes, 0.0,
            ego_to_furthest - current_lane_length + adjacent_lane_range.backward,
            input.route_handler, -0.5, -0.5);
          cut_rear_detection_areas_by_current_lanes(current_lanes, areas);
          connected_adjacent_lanes.clear();
          const auto adjacent_object = assemble_rear_object_from_detection_areas(
            *clustered, areas, input.route_handler);
          if (adjacent_object) {
            const auto relative_distance = adjacent_object->absolute_distance - ego_to_furthest -
              std::abs(input.vehicle_info->min_longitudinal_offset_m);
            if (relative_distance >= pointcloud_dead_zone_ - adjacent_lane_range.forward &&
              !append_safety_object(*adjacent_object, areas, ego_to_furthest,
                adjacent_lane_safety_, false)) {
              object_identity_failure = true;
            }
          }
        }
      }
    }
    if (turn_activation.behavior != RearBehavior::NONE) {
      const bool is_right = turn_activation.behavior == RearBehavior::TURN_RIGHT;
      lanelet::ConstLanelets half_lanes;
      for (const auto & lane : current_lanes) {
        half_lanes.push_back(make_rear_half_lanelet(
          lane, is_right,
          0.5 * input.vehicle_info->vehicle_width_m + 0.1, 0.3));
      }
      std::vector<RearDetectionArea> detection_areas;
      const auto detection_polygon = make_rear_detection_polygon(
        half_lanes, input.current_kinematics->pose.pose,
        blind_spot_range.forward, blind_spot_range.backward);
      if (!detection_polygon.empty()) detection_areas.push_back({detection_polygon, half_lanes});
      const auto blind_spot_object = assemble_rear_object_from_detection_areas(
        *clustered, detection_areas, input.route_handler);
      if (blind_spot_object && !half_lanes.empty()) {
        const auto ego_arc = autoware::experimental::lanelet2_utils::get_arc_coordinates(
          current_lanes, input.current_kinematics->pose.pose).length;
        const auto ego_to_furthest = lanelet::geometry::length2d(
          lanelet::LaneletSequence(half_lanes)) - ego_arc;
        const auto relative_distance = blind_spot_object->absolute_distance - ego_to_furthest -
          std::abs(input.vehicle_info->min_longitudinal_offset_m);
        if (relative_distance >= pointcloud_dead_zone_ - blind_spot_range.forward) {
          auto blind_spot_safety = blind_spot_safety_;
          blind_spot_safety.distance_to_conflict_point = turn_activation.distance_to_conflict;
          if (!append_safety_object(*blind_spot_object, detection_areas, ego_to_furthest,
            blind_spot_safety, true)) {
            object_identity_failure = true;
          }
        }
      }
    }
    if (object_identity_failure) {
      status.is_valid_rear_collision_check = false;
      return;
    }
    const auto objects_safe = rear_objects_are_safe(safety_objects, tracking_config_.observation_time);
    status.is_valid_rear_collision_check = apply_rear_safety_debounce(
      objects_safe, ros_now, on_time_buffer_, off_time_buffer_, safety_debounce_);
    return;
  } catch (const std::exception &) {
    // A malformed cloud is not evidence that the rear area is safe.
    status.is_valid_rear_collision_check = false;
    return;
  }

}

std::string RearCollisionCheckerRos1::module_name() const { return module_name_; }
}  // namespace autoware::planning_validator

PLUGINLIB_EXPORT_CLASS(
  autoware::planning_validator::RearCollisionCheckerRos1,
  autoware::planning_validator::PluginInterface)
