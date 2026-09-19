#pragma once

#include <ros/node_handle.h>

#include <algorithm>
#include <string>

namespace road_user_stop
{
struct Params
{
  struct Option { bool suppress_sudden_stop{true}; } option;
  struct StopPlanning {
    struct LongitudinalMargin { double default_margin{5.0}; double terminal_margin{3.0}; double minimum_margin{3.0}; } longitudinal_margin;
    struct OpposingTraffic { double stop_margin{10.0}; double max_negative_velocity{-0.1}; double min_velocity_for_stop_planning{2.77}; double effective_deceleration{4.0}; } opposing_traffic;
    struct StopOnCurve { bool enable_approaching{false}; double additional_stop_margin{3.0}; double min_stop_margin{6.0}; } stop_on_curve;
    double hold_stop_velocity_threshold{0.01}; double hold_stop_distance_threshold{0.3};
    double limit_min_acc{-2.5}; double sudden_object_acc_threshold{-1.0}; double sudden_object_dist_threshold{1000.0}; bool abandon_to_stop{false};
  } stop_planning;
  struct ObstacleFiltering {
    struct ObjectType { bool pedestrian{true}; bool bicycle{true}; bool motorcycle{false}; bool unknown{false}; } object_type;
    struct OpposingTrafficDetection { bool enable{true}; double angle_threshold{150.0}; double min_speed_threshold{0.5}; } opposing_traffic_detection;
    struct Intersection { bool exclude{false}; } intersection;
    struct Crosswalk { bool exclude{true}; double margin{1.0}; } crosswalk;
    double trajectory_lateral_margin{1.0}; double min_detection_duration{0.1}; double lost_object_retention_duration{2.0}; double polygon_expansion_length{0.5};
  } obstacle_filtering;
  struct StoppedObjectTracking { double stopped_velocity_threshold{0.5}; double stopped_position_tolerance{0.3}; double ego_reached_wall_distance_threshold{0.5}; double stopped_duration_after_ego_arrival{3.0}; } stopped_object_tracking;
};

class ParamListener
{
public:
  explicit ParamListener(ros::NodeHandle & node) : node_(node) { refresh_dynamic_parameters(); }
  Params get_params() const { return params_; }
  void refresh_dynamic_parameters()
  {
    read("road_user_stop.option.suppress_sudden_stop", params_.option.suppress_sudden_stop);
    read("road_user_stop.stop_planning.longitudinal_margin.default_margin", params_.stop_planning.longitudinal_margin.default_margin);
    read("road_user_stop.stop_planning.longitudinal_margin.terminal_margin", params_.stop_planning.longitudinal_margin.terminal_margin);
    read("road_user_stop.stop_planning.longitudinal_margin.minimum_margin", params_.stop_planning.longitudinal_margin.minimum_margin);
    read("road_user_stop.stop_planning.opposing_traffic.stop_margin", params_.stop_planning.opposing_traffic.stop_margin);
    read("road_user_stop.stop_planning.opposing_traffic.max_negative_velocity", params_.stop_planning.opposing_traffic.max_negative_velocity);
    read("road_user_stop.stop_planning.opposing_traffic.min_velocity_for_stop_planning", params_.stop_planning.opposing_traffic.min_velocity_for_stop_planning);
    read("road_user_stop.stop_planning.opposing_traffic.effective_deceleration", params_.stop_planning.opposing_traffic.effective_deceleration);
    read("road_user_stop.stop_planning.hold_stop_velocity_threshold", params_.stop_planning.hold_stop_velocity_threshold);
    read("road_user_stop.stop_planning.hold_stop_distance_threshold", params_.stop_planning.hold_stop_distance_threshold);
    read("road_user_stop.stop_planning.stop_on_curve.enable_approaching", params_.stop_planning.stop_on_curve.enable_approaching);
    read("road_user_stop.stop_planning.stop_on_curve.additional_stop_margin", params_.stop_planning.stop_on_curve.additional_stop_margin);
    read("road_user_stop.stop_planning.stop_on_curve.min_stop_margin", params_.stop_planning.stop_on_curve.min_stop_margin);
    read("road_user_stop.stop_planning.limit_min_acc", params_.stop_planning.limit_min_acc);
    read("road_user_stop.stop_planning.sudden_object_acc_threshold", params_.stop_planning.sudden_object_acc_threshold);
    read("road_user_stop.stop_planning.sudden_object_dist_threshold", params_.stop_planning.sudden_object_dist_threshold);
    read("road_user_stop.stop_planning.abandon_to_stop", params_.stop_planning.abandon_to_stop);
    read("road_user_stop.obstacle_filtering.object_type.pedestrian", params_.obstacle_filtering.object_type.pedestrian);
    read("road_user_stop.obstacle_filtering.object_type.bicycle", params_.obstacle_filtering.object_type.bicycle);
    read("road_user_stop.obstacle_filtering.object_type.motorcycle", params_.obstacle_filtering.object_type.motorcycle);
    read("road_user_stop.obstacle_filtering.object_type.unknown", params_.obstacle_filtering.object_type.unknown);
    read("road_user_stop.obstacle_filtering.trajectory_lateral_margin", params_.obstacle_filtering.trajectory_lateral_margin);
    read("road_user_stop.obstacle_filtering.opposing_traffic_detection.enable", params_.obstacle_filtering.opposing_traffic_detection.enable);
    read("road_user_stop.obstacle_filtering.opposing_traffic_detection.angle_threshold", params_.obstacle_filtering.opposing_traffic_detection.angle_threshold);
    read("road_user_stop.obstacle_filtering.opposing_traffic_detection.min_speed_threshold", params_.obstacle_filtering.opposing_traffic_detection.min_speed_threshold);
    read("road_user_stop.obstacle_filtering.intersection.exclude", params_.obstacle_filtering.intersection.exclude);
    read("road_user_stop.obstacle_filtering.crosswalk.exclude", params_.obstacle_filtering.crosswalk.exclude);
    read("road_user_stop.obstacle_filtering.crosswalk.margin", params_.obstacle_filtering.crosswalk.margin);
    read("road_user_stop.obstacle_filtering.min_detection_duration", params_.obstacle_filtering.min_detection_duration);
    read("road_user_stop.obstacle_filtering.lost_object_retention_duration", params_.obstacle_filtering.lost_object_retention_duration);
    read("road_user_stop.obstacle_filtering.polygon_expansion_length", params_.obstacle_filtering.polygon_expansion_length);
    read("road_user_stop.stopped_object_tracking.stopped_velocity_threshold", params_.stopped_object_tracking.stopped_velocity_threshold);
    read("road_user_stop.stopped_object_tracking.stopped_position_tolerance", params_.stopped_object_tracking.stopped_position_tolerance);
    read("road_user_stop.stopped_object_tracking.ego_reached_wall_distance_threshold", params_.stopped_object_tracking.ego_reached_wall_distance_threshold);
    read("road_user_stop.stopped_object_tracking.stopped_duration_after_ego_arrival", params_.stopped_object_tracking.stopped_duration_after_ego_arrival);
  }
private:
  template <class T> void read(const std::string & dotted, T & value)
  {
    auto slash = dotted; std::replace(slash.begin(), slash.end(), '.', '/');
    node_.getParam(slash, value);
  }
  ros::NodeHandle node_;
  Params params_;
};
}  // namespace road_user_stop
