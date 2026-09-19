#pragma once
#include <ros/ros.h>
#include <autoware/motion_velocity_planner_common/planner_data.hpp>
#include <autoware/motion_velocity_planner_common/plugin_module_interface.hpp>
#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <autoware_utils_rclcpp/parameter.hpp>
#include <stdexcept>
#include <boost/geometry/algorithms/perimeter.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry.hpp>
#include <autoware/motion_velocity_planner_common/polygon_utils.hpp>
namespace autoware::motion_velocity_planner {
namespace polygon_utils {
// Original object timestamp is explicitly unused; preserve its call position.
inline std::optional<std::pair<geometry_msgs::Point,double>> getCollisionPoint(
  const std::vector<autoware_planning_msgs::TrajectoryPoint> & points,
  const std::vector<autoware_utils_geometry::Polygon2d> & polygons,
  const geometry_msgs::Point & position,[[maybe_unused]] const ros::Time & stamp,
  const autoware_utils_geometry::Polygon2d & object,double offset)
{
  assert(points.size()==polygons.size());
  return getCollisionPoint(points,polygons,position,object,offset);
}
}
struct MotionObstacleStopRos1Clock { ros::Time now() const { return ros::Time::now(); } };
namespace motion_obstacle_stop_ros1 {
template<class T> T require_parameter(const ros::NodeHandle & node,const std::string & name) {
  const auto key=autoware_utils_rclcpp::to_ros1_parameter_name(name);
  XmlRpc::XmlRpcValue raw;
  // ROS1 cannot store both scalar 'ns' and 'ns/child'. A reserved leaf
  // represents that scalar only when its namespace also has children.
  if(node.getParam(key,raw) && raw.getType()==XmlRpc::XmlRpcValue::TypeStruct && node.hasParam(key+"/__value"))
    return autoware_utils_rclcpp::detail::strict_get<T>(node,key+"/__value");
  return autoware_utils_rclcpp::detail::strict_get<T>(node,key);
}
template<class T> void stamp(T & out,const ros::Time & now) { out.sec=now.sec;out.nanosec=now.nsec; }
inline void stamp(ros::Time & out,const ros::Time & now) { out=now; }
inline bool update_param(const ParameterMap & parameters,const std::string & key,bool & output) {
  auto it=parameters.find(key);
  if(it==parameters.end()) it=parameters.find(autoware_utils_rclcpp::to_ros1_parameter_name(key));
  if(it==parameters.end()) return false;
  if(it->second.getType()!=XmlRpc::XmlRpcValue::TypeBoolean) throw std::invalid_argument(key+": expected bool");
  output=static_cast<bool>(it->second);return true;
}
}
}
#ifndef RCLCPP_ERROR
#define RCLCPP_ERROR(logger, ...) ROS_ERROR(__VA_ARGS__)
#endif
#ifndef RCLCPP_WARN
#define RCLCPP_WARN(logger, ...) ROS_WARN(__VA_ARGS__)
#endif
#ifndef RCLCPP_DEBUG
#define RCLCPP_DEBUG(logger, ...) ROS_DEBUG(__VA_ARGS__)
#endif
#ifndef RCLCPP_WARN_ONCE
#define RCLCPP_WARN_ONCE(logger, ...) ROS_WARN_ONCE(__VA_ARGS__)
#endif
