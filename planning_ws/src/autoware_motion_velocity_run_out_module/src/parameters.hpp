
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

#ifndef PARAMETERS_HPP_
#define PARAMETERS_HPP_

#include <autoware/universe_utils/ros/parameter.hpp>
#include <autoware/universe_utils/ros/update_param.hpp>
#include "ros1_compat.hpp"
#include <ros/ros.h>
#include <ros/ros.h>

#include <autoware_perception_msgs/ObjectClassification.h>

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

namespace autoware::motion_velocity_planner::run_out
{
/// @brief parameters for the filtering of the dynamic objects
struct ObjectParameters
{
  bool ignore_if_stopped;
  double stopped_velocity_threshold;
  bool ignore_if_on_ego_trajectory;
  bool ignore_if_behind_ego;
  std::vector<std::string> ignore_objects_polygon_types;
  std::vector<std::string> ignore_objects_lanelet_subtypes;
  std::vector<std::string> ignore_collisions_polygon_types;
  std::vector<std::string> ignore_collisions_lanelet_subtypes;
  double start_ignore_collisions_time;
  double start_ignore_collisions_distance;
  std::vector<std::string> cut_linestring_types;
  std::vector<std::string> cut_polygon_types;
  std::vector<std::string> cut_lanelet_subtypes;
  std::vector<std::string> strict_cut_linestring_types;
  std::vector<std::string> strict_cut_polygon_types;
  std::vector<std::string> strict_cut_lanelet_subtypes;
  bool cut_if_crossing_ego_from_behind;
  double confidence_filtering_threshold;
  bool confidence_filtering_only_use_highest;
  double preserved_duration;
  double preserved_distance;
  double standstill_duration_after_cut;
};

/// @brief conditions to ignore collisions
struct IgnoreCollisionConditionsParameters
{
  /// @brief collision where ego arrives first
  struct
  {
    bool enable;
    struct
    {
      std::vector<double>
        ego_enter_times;  // [s] predicted times when ego starts overlapping the path of the object
      std::vector<double>
        time_margins;  // [s] margin values used such that ego needs to enter the overlap before the
                       // object with this much margin to decide to ignore the collision
    } margin;
    double max_overlap_duration;  // [s] the collision is not ignored if ego is predicted to stay on
                                  // the object's path for longer than this duration
  } if_ego_arrives_first;
  /// @brief collision where ego arrives first and cannot stop before the collision
  struct
  {
    bool enable;
    double deceleration_limit;  // [m/s²] deceleration used to determine if ego can stop before a
                                // collision
    double calculated_stop_time_limit;  // [s] calculated from the deceleration limit and the
                                        // current ego velocity
  } if_ego_arrives_first_and_cannot_stop;
};

/// @brief Parameters of the run_out module
struct Parameters
{
  double
    max_history_duration;  // [s]  calculated as the maximum duration among all buffer parameters
  double stop_on_time_buffer;   // [s] successive collision detection time required to start the
                                // stopping decision
  double stop_off_time_buffer;  // [s] successive non-collision detection time required to remove a
                                // stopping decision
  double stop_distance_buffer;  // [m] longitudinal safety distance to keep between ego and the
                                // collision position
  double stop_deceleration_limit;  // [m/s²] if a stop requires a higher deceleration, then an ERROR
                                   // diagnostic is published
  double keep_stop_condition_time;  // [s] time along the ego trajectory where any collision will
                                    // make us keep a stop decision
  double keep_stop_condition_distance;  // [m] distance along the ego trajectory where any collision
                                        // will make us keep a stop decision
  double stop_reuse_margin;  // [m] the previous stop distance is reused if before the newly
                             // calculated one but not further than this margin

  double slowdown_on_time_buffer;      // [s] successive collision detection time required to
                                       // start the slowdown decision
  double slowdown_off_time_buffer;     // [s] successive non-collision detection time
                                       // required to remove the slowdown decision
  double slowdown_distance_buffer;     // [m] longitudinal distance between the collision
                                       // and the slowdown positions
  double slowdown_deceleration_limit;  // [m/s²] maximum deceleration that can be applied
                                       // by the preventive slowdown

  double ego_lateral_margin;                // [m] ego footprint lateral margin
  double ego_longitudinal_margin;           // [m] ego footprint longitudinal margin
  double collision_time_margin;             // [s] extra time margin to determine collisions
  double collision_time_overlap_tolerance;  // [s] when calculating overlap time intervals,
                                            // intervals are grouped if they are separated by less
                                            // than this tolerance value
  double
    collision_same_direction_angle_threshold;  // [rad] threshold to determine if a collision is
                                               // going in the same direction as the ego trajectory
  double collision_opposite_direction_angle_threshold;  // [rad] threshold to determine if a
                                                        // collision is going in a direction
                                                        // opposite to the ego trajectory
  IgnoreCollisionConditionsParameters ignore_collision_conditions;
  // object parameters
  std::vector<std::string> objects_target_labels;
  std::vector<ObjectParameters> object_parameters_per_label;

  struct
  {
    std::string object_label;
    struct
    {
      bool ego_footprint = false;
      bool objects = false;
      bool collisions = false;
      bool decisions = false;
      bool filtering_data = false;
    } enabled_markers;
  } debug;

  /// @brief Get the parameter defined for a specific object label, or the default value if it was
  /// not specified
  template <class T>
  auto get_object_parameter(
    ros::NodeHandle & node, const std::string & ns, const uint8_t object_label,
    const std::string & param)
  {
    using autoware_utils_rclcpp::get_or_declare_parameter;
    const auto label_str = ".objects." + label_to_string(object_label);
    try {
      return get_or_declare_parameter<T>(node, ns + label_str + param);
    } catch (const std::exception &) {
      return get_or_declare_parameter<T>(node, ns + ".objects.DEFAULT" + param);
    }
  }
  /// @brief Initialize the parameters
  void initialize(ros::NodeHandle & node, const std::string & ns)
  {
    using autoware_utils_rclcpp::get_or_declare_parameter;
    const auto ignore_collisions_ns = ns + ".collision.ignore_conditions";
    ignore_collision_conditions.if_ego_arrives_first.enable =
      get_or_declare_parameter<bool>(node, ignore_collisions_ns + ".if_ego_arrives_first.enable");
    ignore_collision_conditions.if_ego_arrives_first.margin.ego_enter_times =
      get_or_declare_parameter<std::vector<double>>(
        node, ignore_collisions_ns + ".if_ego_arrives_first.margin.ego_enter_times");
    ignore_collision_conditions.if_ego_arrives_first.margin.time_margins =
      get_or_declare_parameter<std::vector<double>>(
        node, ignore_collisions_ns + ".if_ego_arrives_first.margin.time_margins");
    ignore_collision_conditions.if_ego_arrives_first.max_overlap_duration =
      get_or_declare_parameter<double>(
        node, ignore_collisions_ns + ".if_ego_arrives_first.max_overlap_duration");
    ignore_collision_conditions.if_ego_arrives_first_and_cannot_stop.enable =
      get_or_declare_parameter<bool>(
        node, ignore_collisions_ns + ".if_ego_arrives_first_and_cannot_stop.enable");
    ignore_collision_conditions.if_ego_arrives_first_and_cannot_stop.deceleration_limit =
      get_or_declare_parameter<double>(
        node, ignore_collisions_ns + ".if_ego_arrives_first_and_cannot_stop.deceleration_limit");
    stop_off_time_buffer = get_or_declare_parameter<double>(node, ns + ".stop.off_time_buffer");
    stop_on_time_buffer = get_or_declare_parameter<double>(node, ns + ".stop.on_time_buffer");
    stop_distance_buffer = get_or_declare_parameter<double>(node, ns + ".stop.distance_buffer");
    stop_deceleration_limit = get_or_declare_parameter<double>(node, ns + ".stop.deceleration_limit");
    keep_stop_condition_time =
      get_or_declare_parameter<double>(node, ns + ".stop.keep_condition.time");
    keep_stop_condition_distance =
      get_or_declare_parameter<double>(node, ns + ".stop.keep_condition.distance");
    stop_reuse_margin = get_or_declare_parameter<double>(node, ns + ".stop.reuse_margin");
    slowdown_off_time_buffer =
      get_or_declare_parameter<double>(node, ns + ".slowdown.off_time_buffer");
    slowdown_on_time_buffer = get_or_declare_parameter<double>(node, ns + ".slowdown.on_time_buffer");
    slowdown_distance_buffer =
      get_or_declare_parameter<double>(node, ns + ".slowdown.distance_buffer");
    slowdown_deceleration_limit =
      get_or_declare_parameter<double>(node, ns + ".slowdown.deceleration_limit");
    ego_lateral_margin = get_or_declare_parameter<double>(node, ns + ".ego.lateral_margin");
    ego_longitudinal_margin = get_or_declare_parameter<double>(node, ns + ".ego.longitudinal_margin");
    collision_time_margin = get_or_declare_parameter<double>(node, ns + ".collision.time_margin");
    collision_time_overlap_tolerance =
      get_or_declare_parameter<double>(node, ns + ".collision.time_overlap_tolerance");
    collision_same_direction_angle_threshold =
      get_or_declare_parameter<double>(node, ns + ".collision.same_direction_angle_threshold");
    collision_opposite_direction_angle_threshold =
      get_or_declare_parameter<double>(node, ns + ".collision.opposite_direction_angle_threshold");

    const auto all_object_labels = all_labels();
    object_parameters_per_label.resize(
      *std::max_element(all_object_labels.begin(), all_object_labels.end()) + 1);
    objects_target_labels =
      get_or_declare_parameter<std::vector<std::string>>(node, ns + ".objects.target_labels");
    for (const auto label : all_object_labels) {
      object_parameters_per_label[label].ignore_if_on_ego_trajectory =
        get_object_parameter<bool>(node, ns, label, ".ignore.if_on_ego_trajectory");
      object_parameters_per_label[label].ignore_if_behind_ego =
        get_object_parameter<bool>(node, ns, label, ".ignore.if_behind_ego");
      object_parameters_per_label[label].ignore_objects_polygon_types =
        get_object_parameter<std::vector<std::string>>(node, ns, label, ".ignore.polygon_types");
      object_parameters_per_label[label].ignore_objects_lanelet_subtypes =
        get_object_parameter<std::vector<std::string>>(node, ns, label, ".ignore.lanelet_subtypes");
      object_parameters_per_label[label].ignore_collisions_polygon_types =
        get_object_parameter<std::vector<std::string>>(
          node, ns, label, ".ignore_collisions.polygon_types");
      object_parameters_per_label[label].ignore_collisions_lanelet_subtypes =
        get_object_parameter<std::vector<std::string>>(
          node, ns, label, ".ignore_collisions.lanelet_subtypes");
      object_parameters_per_label[label].start_ignore_collisions_time =
        get_object_parameter<double>(node, ns, label, ".ignore_collisions.start_ignore_time");
      object_parameters_per_label[label].start_ignore_collisions_distance =
        get_object_parameter<double>(node, ns, label, ".ignore_collisions.start_ignore_distance");
      object_parameters_per_label[label].ignore_if_stopped =
        get_object_parameter<bool>(node, ns, label, ".ignore.if_stopped");
      object_parameters_per_label[label].stopped_velocity_threshold =
        get_object_parameter<double>(node, ns, label, ".ignore.stopped_velocity_threshold");
      object_parameters_per_label[label].confidence_filtering_threshold =
        get_object_parameter<double>(node, ns, label, ".confidence_filtering.threshold");
      object_parameters_per_label[label].confidence_filtering_only_use_highest =
        get_object_parameter<bool>(node, ns, label, ".confidence_filtering.only_use_highest");
      object_parameters_per_label[label].cut_polygon_types =
        get_object_parameter<std::vector<std::string>>(
          node, ns, label, ".cut_predicted_paths.polygon_types");
      object_parameters_per_label[label].cut_lanelet_subtypes =
        get_object_parameter<std::vector<std::string>>(
          node, ns, label, ".cut_predicted_paths.lanelet_subtypes");
      object_parameters_per_label[label].cut_linestring_types =
        get_object_parameter<std::vector<std::string>>(
          node, ns, label, ".cut_predicted_paths.linestring_types");
      object_parameters_per_label[label].strict_cut_polygon_types =
        get_object_parameter<std::vector<std::string>>(
          node, ns, label, ".cut_predicted_paths.strict_polygon_types");
      object_parameters_per_label[label].strict_cut_lanelet_subtypes =
        get_object_parameter<std::vector<std::string>>(
          node, ns, label, ".cut_predicted_paths.strict_lanelet_subtypes");
      object_parameters_per_label[label].strict_cut_linestring_types =
        get_object_parameter<std::vector<std::string>>(
          node, ns, label, ".cut_predicted_paths.strict_linestring_types");
      object_parameters_per_label[label].cut_if_crossing_ego_from_behind =
        get_object_parameter<bool>(
          node, ns, label, ".cut_predicted_paths.if_crossing_ego_from_behind");
      object_parameters_per_label[label].preserved_duration =
        get_object_parameter<double>(node, ns, label, ".preserved_duration");
      object_parameters_per_label[label].preserved_distance =
        get_object_parameter<double>(node, ns, label, ".preserved_distance");
      object_parameters_per_label[label].standstill_duration_after_cut =
        get_object_parameter<double>(node, ns, label, ".standstill_duration_after_cut");
    }
    debug.object_label = get_or_declare_parameter<std::string>(node, ns + ".debug.object_label");
    debug.enabled_markers.ego_footprint =
      get_or_declare_parameter<bool>(node, ns + ".debug.enabled_markers.ego_footprint");
    debug.enabled_markers.objects =
      get_or_declare_parameter<bool>(node, ns + ".debug.enabled_markers.objects");
    debug.enabled_markers.collisions =
      get_or_declare_parameter<bool>(node, ns + ".debug.enabled_markers.collisions");
    debug.enabled_markers.decisions =
      get_or_declare_parameter<bool>(node, ns + ".debug.enabled_markers.decisions");
    debug.enabled_markers.filtering_data =
      get_or_declare_parameter<bool>(node, ns + ".debug.enabled_markers.filtering_data");

    max_history_duration = std::max(stop_off_time_buffer, stop_on_time_buffer);
  }
  /// @brief Update the parameters
  void update(const ParameterMap & params, const std::string & ns)
  {
    using autoware_utils::update_param;
    update_param(
      params, ns + ".collision.ignore_conditions.if_ego_arrives_first.enable",
      ignore_collision_conditions.if_ego_arrives_first.enable);
    update_param(
      params, ns + ".collision.ignore_conditions.if_ego_arrives_first.margin.ego_enter_times",
      ignore_collision_conditions.if_ego_arrives_first.margin.ego_enter_times);
    update_param(
      params, ns + ".collision.ignore_conditions.if_ego_arrives_first.margin.time_margins",
      ignore_collision_conditions.if_ego_arrives_first.margin.time_margins);
    update_param(
      params, ns + ".collision.ignore_conditions.if_ego_arrives_first.max_overlap_duration",
      ignore_collision_conditions.if_ego_arrives_first.max_overlap_duration);
    update_param(
      params, ns + ".collision.if_ego_arrives_first_and_cannot_stop.enable",
      ignore_collision_conditions.if_ego_arrives_first_and_cannot_stop.enable);
    update_param(
      params, ns + ".collision.if_ego_arrives_first_and_cannot_stop.deceleration_limit",
      ignore_collision_conditions.if_ego_arrives_first_and_cannot_stop.deceleration_limit);
    update_param(params, ns + ".slowdown.on_time_buffer", slowdown_on_time_buffer);
    update_param(params, ns + ".slowdown.off_time_buffer", slowdown_off_time_buffer);
    update_param(params, ns + ".slowdown.distance_buffer", slowdown_distance_buffer);
    update_param(params, ns + ".slowdown.deceleration_limit", slowdown_deceleration_limit);
    update_param(params, ns + ".stop.on_time_buffer", stop_on_time_buffer);
    update_param(params, ns + ".stop.off_time_buffer", stop_off_time_buffer);
    update_param(params, ns + ".stop.distance_buffer", stop_distance_buffer);
    update_param(params, ns + ".stop.keep_condition.time", keep_stop_condition_time);
    update_param(params, ns + ".stop.keep_condition.distance", keep_stop_condition_distance);
    update_param(params, ns + ".stop.reuse_margin", stop_reuse_margin);
    update_param(params, ns + ".ego.lateral_margin", ego_lateral_margin);
    update_param(params, ns + ".ego.longitudinal_margin", ego_longitudinal_margin);
    update_param(params, ns + ".collision.time_margin", collision_time_margin);
    update_param(params, ns + ".collision.time_overlap_tolerance", collision_time_overlap_tolerance);
    update_param(
      params, ns + ".collision.same_direction_angle_threshold",
      collision_same_direction_angle_threshold);
    update_param(
      params, ns + ".collision.opposite_direction_angle_threshold",
      collision_opposite_direction_angle_threshold);

    update_param(params, ns + ".objects.target_labels", objects_target_labels);

    for (const auto label : all_labels()) {
      const auto str = ".objects." + label_to_string(label);
      update_param(
        params, ns + str + ".ignore.if_stopped",
        object_parameters_per_label[label].ignore_if_stopped);
      update_param(
        params, ns + str + ".ignore.stopped_velocity_threshold",
        object_parameters_per_label[label].stopped_velocity_threshold);
      update_param(
        params, ns + str + ".ignore.if_on_ego_trajectory",
        object_parameters_per_label[label].ignore_if_on_ego_trajectory);
      update_param(
        params, ns + str + ".ignore.if_behind_ego",
        object_parameters_per_label[label].ignore_if_behind_ego);
      update_param(
        params, ns + str + ".ignore.lanelet_subtypes",
        object_parameters_per_label[label].ignore_objects_lanelet_subtypes);
      update_param(
        params, ns + str + ".ignore_collisions.lanelet_subtypes",
        object_parameters_per_label[label].ignore_collisions_lanelet_subtypes);
      update_param(
        params, ns + str + ".ignore.polygon_types",
        object_parameters_per_label[label].ignore_objects_polygon_types);
      update_param(
        params, ns + str + ".ignore_collisions.polygon_types",
        object_parameters_per_label[label].ignore_collisions_polygon_types);
      update_param(
        params, ns + str + ".ignore_collisions.start_ignore_time",
        object_parameters_per_label[label].start_ignore_collisions_time);
      update_param(
        params, ns + str + ".ignore_collisions.start_ignore_distance",
        object_parameters_per_label[label].start_ignore_collisions_distance);
      update_param(
        params, ns + str + ".confidence_filtering.threshold",
        object_parameters_per_label[label].confidence_filtering_threshold);
      update_param(
        params, ns + str + ".confidence_filtering.only_use_highest",
        object_parameters_per_label[label].confidence_filtering_only_use_highest);
      update_param(
        params, ns + str + ".cut_predicted_paths.if_crossing_ego_from_behind",
        object_parameters_per_label[label].cut_if_crossing_ego_from_behind);
      update_param(
        params, ns + str + ".cut_predicted_paths.strict_lanelet_subtypes",
        object_parameters_per_label[label].strict_cut_lanelet_subtypes);
      update_param(
        params, ns + str + ".cut_predicted_paths.strict_polygon_types",
        object_parameters_per_label[label].strict_cut_polygon_types);
      update_param(
        params, ns + str + ".cut_predicted_paths.strict_linestring_types",
        object_parameters_per_label[label].strict_cut_linestring_types);
      update_param(
        params, ns + str + ".standstill_duration_after_cut",
        object_parameters_per_label[label].standstill_duration_after_cut);
      update_param(
        params, ns + str + ".preserved_duration",
        object_parameters_per_label[label].preserved_duration);
      update_param(
        params, ns + str + ".preserved_distance",
        object_parameters_per_label[label].preserved_distance);
    }
    update_param(params, ns + ".debug.object_label", debug.object_label);
    update_param(
      params, ns + ".debug.enabled_markers.ego_footprint", debug.enabled_markers.ego_footprint);
    update_param(params, ns + ".debug.enabled_markers.objects", debug.enabled_markers.objects);
    update_param(params, ns + ".debug.enabled_markers.collisions", debug.enabled_markers.collisions);
    update_param(params, ns + ".debug.enabled_markers.decisions", debug.enabled_markers.decisions);
    update_param(
      params, ns + ".debug.enabled_markers.filtering_data", debug.enabled_markers.filtering_data);

    max_history_duration = std::max(stop_off_time_buffer, stop_on_time_buffer);
  }
  /// @brief get a string representation of the given classification label
  static std::string label_to_string(
    const autoware_perception_msgs::ObjectClassification::_label_type & label)
  {
    using autoware_perception_msgs::ObjectClassification;
    switch (label) {
      case ObjectClassification::CAR:
        return "CAR";
      case ObjectClassification::TRUCK:
        return "TRUCK";
      case ObjectClassification::BICYCLE:
        return "BICYCLE";
      case ObjectClassification::BUS:
        return "BUS";
      case ObjectClassification::MOTORCYCLE:
        return "MOTORCYCLE";
      case ObjectClassification::PEDESTRIAN:
        return "PEDESTRIAN";
      case ObjectClassification::TRAILER:
        return "TRAILER";
      case ObjectClassification::UNKNOWN:
        return "UNKNOWN";
      default:
        return "DEFAULT";
    }
  }
  /// @brief get the label corresponding to the given string
  static autoware_perception_msgs::ObjectClassification::_label_type string_to_label(
    const std::string & label)
  {
    using autoware_perception_msgs::ObjectClassification;
    if (label == "CAR") {
      return ObjectClassification::CAR;
    }
    if (label == "TRUCK") {
      return ObjectClassification::TRUCK;
    }
    if (label == "BICYCLE") {
      return ObjectClassification::BICYCLE;
    }
    if (label == "BUS") {
      return ObjectClassification::BUS;
    }
    if (label == "MOTORCYCLE") {
      return ObjectClassification::MOTORCYCLE;
    }
    if (label == "PEDESTRIAN") {
      return ObjectClassification::PEDESTRIAN;
    }
    if (label == "TRAILER") {
      return ObjectClassification::TRAILER;
    }
    return ObjectClassification::UNKNOWN;
  }
  [[nodiscard]] std::vector<uint8_t> target_labels() const
  {
    using autoware_perception_msgs::ObjectClassification;
    std::vector<uint8_t> labels;
    for (uint8_t label = ObjectClassification::UNKNOWN; label <= ObjectClassification::PEDESTRIAN;
         ++label) {
      const auto is_target_label =
        std::find(
          objects_target_labels.begin(), objects_target_labels.end(),
          Parameters::label_to_string(label)) != objects_target_labels.end();
      if (is_target_label) {
        labels.push_back(label);
      }
    }
    return labels;
  }

  /// @brief get all possible classification labels
  static std::vector<uint8_t> all_labels()
  {
    using autoware_perception_msgs::ObjectClassification;
    std::vector<uint8_t> labels;
    for (uint8_t label = ObjectClassification::UNKNOWN; label <= ObjectClassification::PEDESTRIAN;
         ++label) {
      labels.push_back(label);
    }
    return labels;
  }
};
}  // namespace autoware::motion_velocity_planner::run_out

#endif  // PARAMETERS_HPP_
