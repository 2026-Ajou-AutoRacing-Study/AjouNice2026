#pragma once
#include <autoware_utils_rclcpp/parameter.hpp>
#include <cmath>
#include <stdexcept>
namespace path_generator {
struct Params {
  double planning_hz{};
  struct {
  double backward{};
  double forward{};
  } path_length;
  struct {
  double connection_gradient_from_centerline{};
  } waypoint;
  struct {
  double search_time{};
  double search_distance{};
  double resampling_interval{};
  double angle_threshold_deg{};
  } turn_signal;
  struct {
  double connection_section_length{};
  double pre_goal_offset{};
  } goal_connection;
  double ego_nearest_dist_threshold{};
  double ego_nearest_yaw_threshold{};
};
class ParamListener {
  ros::NodeHandle node_;
  Params cached_;
  Params read() const {
    Params p;
    p.planning_hz=autoware_utils_rclcpp::detail::strict_get<double>(node_,"planning_hz");
    if(!std::isfinite(p.planning_hz) || !(p.planning_hz > 0)) throw std::invalid_argument("planning_hz: original schema bound");
    p.path_length.backward=autoware_utils_rclcpp::detail::strict_get<double>(node_,"path_length/backward");
    if(!std::isfinite(p.path_length.backward) || !(p.path_length.backward > 0)) throw std::invalid_argument("path_length/backward: original schema bound");
    p.path_length.forward=autoware_utils_rclcpp::detail::strict_get<double>(node_,"path_length/forward");
    if(!std::isfinite(p.path_length.forward) || !(p.path_length.forward > 0)) throw std::invalid_argument("path_length/forward: original schema bound");
    p.waypoint.connection_gradient_from_centerline=autoware_utils_rclcpp::detail::strict_get<double>(node_,"waypoint/connection_gradient_from_centerline");
    if(!std::isfinite(p.waypoint.connection_gradient_from_centerline) || !(p.waypoint.connection_gradient_from_centerline > 0)) throw std::invalid_argument("waypoint/connection_gradient_from_centerline: original schema bound");
    p.turn_signal.search_time=autoware_utils_rclcpp::detail::strict_get<double>(node_,"turn_signal/search_time");
    if(!std::isfinite(p.turn_signal.search_time) || !(p.turn_signal.search_time >= 0)) throw std::invalid_argument("turn_signal/search_time: original schema bound");
    p.turn_signal.search_distance=autoware_utils_rclcpp::detail::strict_get<double>(node_,"turn_signal/search_distance");
    if(!std::isfinite(p.turn_signal.search_distance) || !(p.turn_signal.search_distance > 0)) throw std::invalid_argument("turn_signal/search_distance: original schema bound");
    p.turn_signal.resampling_interval=autoware_utils_rclcpp::detail::strict_get<double>(node_,"turn_signal/resampling_interval");
    if(!std::isfinite(p.turn_signal.resampling_interval) || !(p.turn_signal.resampling_interval > 0)) throw std::invalid_argument("turn_signal/resampling_interval: original schema bound");
    p.turn_signal.angle_threshold_deg=autoware_utils_rclcpp::detail::strict_get<double>(node_,"turn_signal/angle_threshold_deg");
    if(!std::isfinite(p.turn_signal.angle_threshold_deg) || !(p.turn_signal.angle_threshold_deg > 0)) throw std::invalid_argument("turn_signal/angle_threshold_deg: original schema bound");
    p.goal_connection.connection_section_length=autoware_utils_rclcpp::detail::strict_get<double>(node_,"goal_connection/connection_section_length");
    if(!std::isfinite(p.goal_connection.connection_section_length) || !(p.goal_connection.connection_section_length > 0)) throw std::invalid_argument("goal_connection/connection_section_length: original schema bound");
    p.goal_connection.pre_goal_offset=autoware_utils_rclcpp::detail::strict_get<double>(node_,"goal_connection/pre_goal_offset");
    if(!std::isfinite(p.goal_connection.pre_goal_offset) || !(p.goal_connection.pre_goal_offset > 0)) throw std::invalid_argument("goal_connection/pre_goal_offset: original schema bound");
    p.ego_nearest_dist_threshold=autoware_utils_rclcpp::detail::strict_get<double>(node_,"ego_nearest_dist_threshold");
    if(!std::isfinite(p.ego_nearest_dist_threshold) || !(p.ego_nearest_dist_threshold > 0)) throw std::invalid_argument("ego_nearest_dist_threshold: original schema bound");
    p.ego_nearest_yaw_threshold=autoware_utils_rclcpp::detail::strict_get<double>(node_,"ego_nearest_yaw_threshold");
    if(!std::isfinite(p.ego_nearest_yaw_threshold) || !(p.ego_nearest_yaw_threshold > 0)) throw std::invalid_argument("ego_nearest_yaw_threshold: original schema bound");
    return p;
  }
public:
  explicit ParamListener(const ros::NodeHandle & node):node_(node),cached_(read()) {}
  Params get_params() {
    try { cached_=read(); }
    catch(const std::exception & error) { ROS_ERROR_THROTTLE(5.0,"Path Generator parameters rejected, retaining last valid values: %s",error.what()); }
    return cached_;
  }
};
}
