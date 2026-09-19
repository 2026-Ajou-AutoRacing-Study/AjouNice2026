#include "autoware/planning_validator/debug_marker_ros1.hpp"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace autoware::planning_validator
{
DebugMarkerRos1::DebugMarkerRos1(ros::NodeHandle private_nh)
{
  marker_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("debug/marker", 1);
  virtual_wall_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("virtual_wall", 1);
}

std::string DebugMarkerRos1::status_string(const autoware_planning_validator::PlanningValidatorStatus & s)
{
  std::string result;
  const auto append = [&result](const char * name, const bool valid) {
    if (!valid) result += (result.empty() ? "[" : " | ") + std::string{name};
  };
  append("latency", s.is_valid_latency); append("traj_size", s.is_valid_size);
  append("infinite_value", s.is_valid_finite_value); append("interval", s.is_valid_interval);
  append("relative_angle", s.is_valid_relative_angle); append("distance_deviation", s.is_valid_distance_deviation);
  append("lon_distance_deviation", s.is_valid_longitudinal_distance_deviation);
  append("velocity_deviation", s.is_valid_velocity_deviation);
  append("lon_max_acc", s.is_valid_longitudinal_max_acc); append("lon_min_acc", s.is_valid_longitudinal_min_acc);
  append("lat_acc", s.is_valid_lateral_acc); append("lat_jerk", s.is_valid_lateral_jerk);
  append("yaw_deviation", s.is_valid_yaw_deviation); append("curvature", s.is_valid_curvature);
  append("steering", s.is_valid_steering); append("steering_rate", s.is_valid_steering_rate);
  append("forward_traj_length", s.is_valid_forward_trajectory_length);
  append("traj_shift", s.is_valid_trajectory_shift); append("collision", s.is_valid_intersection_collision_check);
  return result.empty() ? result : result + "]";
}

void DebugMarkerRos1::publish_invalid(
  const geometry_msgs::Pose & ego_pose, const autoware_planning_validator::PlanningValidatorStatus & status)
{
  visualization_msgs::Marker wall;
  wall.header.frame_id = "map"; wall.header.stamp = ros::Time::now(); wall.ns = "autoware_planning_validator";
  wall.id = 0; wall.type = visualization_msgs::Marker::CUBE; wall.action = visualization_msgs::Marker::ADD;
  wall.pose = ego_pose; wall.pose.position.x += 3.0; wall.scale.x = 0.2; wall.scale.y = 3.0; wall.scale.z = 2.0;
  wall.color.r = 1.0F; wall.color.a = 0.999F; wall.lifetime = ros::Duration(0.2);
  visualization_msgs::Marker text = wall;
  text.ns = "warning_msg"; text.id = 1; text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text.pose.position.x += 0.5; text.pose.position.z += 1.5; text.scale.x = text.scale.y = 0.0; text.scale.z = 1.0;
  text.text = "INVALID PLANNING" + status_string(status);
  visualization_msgs::MarkerArray wall_array; wall_array.markers = {wall, text};
  virtual_wall_pub_.publish(wall_array);
  visualization_msgs::MarkerArray debug_array; debug_array.markers = {wall}; marker_pub_.publish(debug_array);
}
}  // namespace autoware::planning_validator
