#ifndef AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__PLANNER_DATA_HPP_
#define AUTOWARE_BEHAVIOR_PATH_PLANNER_COMMON__PLANNER_DATA_HPP_

#include <autoware/behavior_path_planner_common/parameters.hpp>
#include <autoware/behavior_path_planner_common/avoidance_object_ownership.hpp>
#include <autoware/behavior_path_planner_common/turn_signal_decider.hpp>
#include <autoware/behavior_path_planner_common/utils/drivable_area_expansion/parameters.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <autoware_adapi_v1_msgs/OperationModeState.h>
#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <autoware_internal_planning_msgs/VelocityLimit.h>
#include <autoware_perception_msgs/PredictedObjects.h>
#include <autoware_perception_msgs/TrafficLightGroup.h>
#include <autoware_perception_msgs/TrafficLightGroupArray.h>
#include <autoware_planning_msgs/PoseWithUuidStamped.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tier4_planning_msgs/LateralOffset.h>
#include <unique_identifier_msgs/UUID.h>

#include <algorithm>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::route_handler { class RouteHandler; }

namespace autoware::behavior_path_planner
{
struct TrafficSignalStamped
{
  ros::Time stamp;
  autoware_perception_msgs::TrafficLightGroup signal;
};

// ROS1 owner for the upstream PlannerData message state.  Turn-signal planning methods
// are deliberately kept out until TurnSignalDecider itself is ported.
struct PlannerData
{
  nav_msgs::OdometryConstPtr self_odometry;
  geometry_msgs::AccelWithCovarianceStampedConstPtr self_acceleration;
  autoware_perception_msgs::PredictedObjectsConstPtr dynamic_object;
  nav_msgs::OccupancyGridConstPtr occupancy_grid;
  nav_msgs::OccupancyGridConstPtr costmap;
  tier4_planning_msgs::LateralOffsetConstPtr lateral_offset;
  autoware_adapi_v1_msgs::OperationModeStateConstPtr operation_mode;
  autoware_internal_planning_msgs::PathWithLaneIdPtr prev_output_path{
    boost::make_shared<autoware_internal_planning_msgs::PathWithLaneId>()};
  std::optional<autoware_planning_msgs::PoseWithUuidStamped> prev_modified_goal;
  std::optional<unique_identifier_msgs::UUID> prev_route_id;
  std::shared_ptr<autoware::route_handler::RouteHandler> route_handler;
  std::map<int64_t, TrafficSignalStamped> traffic_light_id_map;
  BehaviorPathPlannerParameters parameters;
  mutable TurnSignalDecider turn_signal_decider;
  drivable_area_expansion::DrivableAreaExpansionParameters drivable_area_expansion_parameters;
  mutable std::vector<geometry_msgs::Pose> drivable_area_expansion_prev_path_poses;
  mutable std::vector<double> drivable_area_expansion_prev_curvatures;
  autoware_internal_planning_msgs::VelocityLimitConstPtr external_limit_max_velocity;
  std::shared_ptr<AvoidanceObjectOwnership> avoidance_object_ownership;

  // ROS1 equivalent of the upstream rclcpp parameter initialization. Required
  // values fail closed, so callers must load the original parameter documents.
  void init_parameters(const ros::NodeHandle & node)
  {
    std::string avoidance_method;
    node.param<std::string>("avoidance/path_generation_method", avoidance_method, "");
    if (avoidance_method == "optimization_base") {
      double dynamic_speed, static_speed, pedestrian_dynamic_speed, pedestrian_static_speed;
      node.param("avoidance_object_ownership/dynamic_enter_speed", dynamic_speed, 0.5);
      node.param("avoidance_object_ownership/static_enter_speed", static_speed, 0.2);
      node.param("avoidance_object_ownership/pedestrian_dynamic_enter_speed", pedestrian_dynamic_speed, 0.7);
      node.param("avoidance_object_ownership/pedestrian_static_enter_speed", pedestrian_static_speed, 0.2);
      avoidance_object_ownership =
        std::make_shared<AvoidanceObjectOwnership>(
          dynamic_speed, static_speed, pedestrian_dynamic_speed, pedestrian_static_speed);
    }
    const auto number = [&node](const std::string & key) {
      double value{};
      auto slash = key; std::replace(slash.begin(), slash.end(), '.', '/');
      if (node.getParam(slash, value)) return value;
      throw std::runtime_error("missing BPP parameter: " + key);
    };
    const auto boolean = [&node](const std::string & key) {
      bool value{};
      auto slash = key; std::replace(slash.begin(), slash.end(), '.', '/');
      if (node.getParam(slash, value)) return value;
      throw std::runtime_error("missing BPP parameter: " + key);
    };
    const auto text = [&node](const std::string & key) {
      std::string value;
      auto slash = key; std::replace(slash.begin(), slash.end(), '.', '/');
      if (node.getParam(slash, value)) return value;
      throw std::runtime_error("missing BPP parameter: " + key);
    };
    const auto integer = [&node](const std::string & key) {
      int value{};
      auto slash = key; std::replace(slash.begin(), slash.end(), '.', '/');
      if (node.getParam(slash, value)) return value;
      throw std::runtime_error("missing BPP parameter: " + key);
    };
    const auto vehicle = autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo();
    parameters.vehicle_info = vehicle;
    parameters.vehicle_width = vehicle.vehicle_width_m; parameters.vehicle_length = vehicle.vehicle_length_m;
    parameters.wheel_tread = vehicle.wheel_tread_m; parameters.wheel_base = vehicle.wheel_base_m;
    parameters.front_overhang = vehicle.front_overhang_m; parameters.rear_overhang = vehicle.rear_overhang_m;
    parameters.left_over_hang = vehicle.left_overhang_m; parameters.right_over_hang = vehicle.right_overhang_m;
    parameters.base_link2front = vehicle.max_longitudinal_offset_m; parameters.base_link2rear = vehicle.rear_overhang_m;
    parameters.traffic_light_signal_timeout = number("traffic_light_signal_timeout");
    parameters.backward_path_length = number("backward_path_length") + vehicle.rear_overhang_m + 1.0;
    parameters.forward_path_length = number("forward_path_length"); parameters.min_acc = number("normal.min_acc");
    parameters.max_acc = number("normal.max_acc"); parameters.max_vel = number("max_vel");
    parameters.backward_length_buffer_for_end_of_pull_over = number("backward_length_buffer_for_end_of_pull_over");
    parameters.backward_length_buffer_for_end_of_pull_out = number("backward_length_buffer_for_end_of_pull_out");
    parameters.minimum_pull_over_length = number("minimum_pull_over_length");
    parameters.refine_goal_search_radius_range = number("refine_goal_search_radius_range");
    parameters.turn_signal_intersection_search_distance = number("turn_signal_intersection_search_distance");
    parameters.turn_signal_intersection_angle_threshold_deg = number("turn_signal_intersection_angle_threshold_deg");
    parameters.turn_signal_minimum_search_distance = number("turn_signal_minimum_search_distance");
    parameters.turn_signal_search_time = number("turn_signal_search_time");
    parameters.turn_signal_shift_length_threshold = number("turn_signal_shift_length_threshold");
    parameters.turn_signal_remaining_shift_length_threshold = number("turn_signal_remaining_shift_length_threshold");
    parameters.turn_signal_remaining_distance_to_bound_threshold = number("turn_signal_remaining_distance_to_bound_threshold");
    parameters.turn_signal_on_swerving = boolean("turn_signal_on_swerving");
    parameters.turn_signal_roundabout_on_entry = text("turn_signal_roundabout_on_entry");
    parameters.turn_signal_roundabout_on_exit = text("turn_signal_roundabout_on_exit");
    parameters.turn_signal_roundabout_entry_indicator_persistence = boolean("turn_signal_roundabout_entry_indicator_persistence");
    parameters.turn_signal_roundabout_search_distance = number("turn_signal_roundabout_search_distance");
    parameters.turn_signal_roundabout_angle_threshold_deg = number("turn_signal_roundabout_angle_threshold_deg");
    parameters.turn_signal_roundabout_backward_depth = integer("turn_signal_roundabout_backward_depth");
    parameters.turn_signal_path_backward_length = number("turn_signal_path_backward_length");
    parameters.enable_akima_spline_first = boolean("enable_akima_spline_first");
    parameters.enable_cog_on_centerline = boolean("enable_cog_on_centerline");
    parameters.input_path_interval = number("input_path_interval"); parameters.output_path_interval = number("output_path_interval");
    parameters.ego_nearest_dist_threshold = number("ego_nearest_dist_threshold");
    parameters.ego_nearest_yaw_threshold = number("ego_nearest_yaw_threshold");
    drivable_area_expansion_parameters.init(node);
  }

  autoware_vehicle_msgs::TurnIndicatorsCommand getTurnSignal(
    const autoware_internal_planning_msgs::PathWithLaneId & path,
    const TurnSignalInfo & info, TurnSignalDebugData & debug)
  {
    return turn_signal_decider.getTurnSignal(route_handler,path,info,self_odometry->pose.pose,
      self_odometry->twist.twist.linear.x,parameters,debug);
  }
  void ensureRouteHandler();
  void updateTrafficSignals(const autoware_perception_msgs::TrafficLightGroupArray & message);
  [[nodiscard]] std::optional<TrafficSignalStamped> getTrafficSignal(
    int64_t id, const ros::Time & now) const;

  template <class T>
  size_t findEgoIndex(const std::vector<T> & points) const
  {
    return autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
      points, self_odometry->pose.pose, parameters.ego_nearest_dist_threshold,
      parameters.ego_nearest_yaw_threshold);
  }

  template <class T>
  size_t findEgoSegmentIndex(const std::vector<T> & points) const
  {
    return autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      points, self_odometry->pose.pose, parameters.ego_nearest_dist_threshold,
      parameters.ego_nearest_yaw_threshold);
  }

  // Original PathWithLaneId overload used by the full Start Planner.
  std::pair<TurnSignalInfo, bool> getBehaviorTurnSignalInfo(
    const autoware_internal_planning_msgs::PathWithLaneId & path,
    size_t start_index, size_t end_index, const lanelet::ConstLanelets & lanes,
    double current_shift, bool forward, bool shifted, bool override_stopped = false,
    bool pull_out = false, bool lane_change = false, bool pull_over = false) const
  {
    if (start_index >= path.points.size() || end_index >= path.points.size()) {
      ROS_WARN("getBehaviorTurnSignalInfo: index inconsistency.");
      return {TurnSignalInfo{}, true};
    }
    ShiftedPath shifted_path{path, std::vector<double>(path.points.size(), 0.0)};
    ShiftLine line;
    line.start = path.points.at(start_index).point.pose;
    line.end = path.points.at(end_index).point.pose;
    line.start_shift_length = autoware::experimental::lanelet2_utils::get_arc_coordinates(
      lanes, line.start).distance;
    line.end_shift_length = autoware::experimental::lanelet2_utils::get_arc_coordinates(
      lanes, line.end).distance;
    shifted_path.shift_length.at(start_index) = line.start_shift_length;
    shifted_path.shift_length.at(end_index) = line.end_shift_length;
    line.start_idx = start_index;
    line.end_idx = end_index;
    return turn_signal_decider.getBehaviorTurnSignalInfo(
      shifted_path, line, lanes, route_handler, parameters, self_odometry,
      parameters.vehicle_info, current_shift, forward, shifted,
      override_stopped, pull_out, lane_change, pull_over);
  }

  std::pair<TurnSignalInfo, bool> getBehaviorTurnSignalInfo(
    const ShiftedPath & path, const ShiftLine & shift_line,
    const lanelet::ConstLanelets & current_lanelets, const double current_shift_length,
    const bool is_driving_forward, const bool egos_lane_is_shifted,
    const bool override_ego_stopped_check = false, const bool is_pull_out = false,
    const bool is_lane_change = false, const bool is_pull_over = false) const
  {
    return turn_signal_decider.getBehaviorTurnSignalInfo(
      path, shift_line, current_lanelets, route_handler, parameters, self_odometry,
      parameters.vehicle_info, current_shift_length, is_driving_forward, egos_lane_is_shifted,
      override_ego_stopped_check, is_pull_out, is_lane_change, is_pull_over);
  }
};
}  // namespace autoware::behavior_path_planner

#endif
