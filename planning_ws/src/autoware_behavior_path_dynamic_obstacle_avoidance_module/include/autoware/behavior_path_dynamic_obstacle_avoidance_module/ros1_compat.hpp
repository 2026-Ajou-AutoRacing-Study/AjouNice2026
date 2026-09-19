#pragma once

// Candidate-local ROS2 spelling compatibility boundary.  This is deliberately
// not exported by a shared support package: it makes every ROS1 type mapping
// used by Dynamic Obstacle Avoidance reviewable in this candidate source.
#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <autoware_perception_msgs/ObjectClassification.h>
#include <autoware_perception_msgs/PredictedObject.h>
#include <autoware_perception_msgs/PredictedPath.h>
#include <autoware_perception_msgs/Shape.h>
#include <autoware_vehicle_msgs/TurnIndicatorsCommand.h>
#include <autoware_utils_visualization/marker_helper.hpp>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <rclcpp/rclcpp.hpp>
#include <ros/ros.h>
#include <tier4_planning_msgs/AvoidanceDebugMsg.h>
#include <tier4_planning_msgs/AvoidanceDebugMsgArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

namespace geometry_msgs { namespace msg {
using Point = ::geometry_msgs::Point;
using Pose = ::geometry_msgs::Pose;
}}  // namespace geometry_msgs::msg
namespace visualization_msgs { namespace msg {
using Marker = ::visualization_msgs::Marker;
using MarkerArray = ::visualization_msgs::MarkerArray;
}}  // namespace visualization_msgs::msg
namespace autoware_internal_planning_msgs { namespace msg {
using PathWithLaneId = ::autoware_internal_planning_msgs::PathWithLaneId;
}}  // namespace autoware_internal_planning_msgs::msg
namespace autoware_perception_msgs { namespace msg {
using ObjectClassification = ::autoware_perception_msgs::ObjectClassification;
using PredictedObject = ::autoware_perception_msgs::PredictedObject;
using PredictedPath = ::autoware_perception_msgs::PredictedPath;
using Shape = ::autoware_perception_msgs::Shape;
}}  // namespace autoware_perception_msgs::msg
namespace autoware_vehicle_msgs { namespace msg {
using TurnIndicatorsCommand = ::autoware_vehicle_msgs::TurnIndicatorsCommand;
}}  // namespace autoware_vehicle_msgs::msg
namespace tier4_planning_msgs { namespace msg {
using AvoidanceDebugMsg = ::tier4_planning_msgs::AvoidanceDebugMsg;
using AvoidanceDebugMsgArray = ::tier4_planning_msgs::AvoidanceDebugMsgArray;
}}  // namespace tier4_planning_msgs::msg

namespace rclcpp
{
// Common ROS2-spelling types and parameter-event decoding have one owner in
// the installed AW149 facade.  Keep only this module-specific ROS1 adapter.
class Node
{
public:
  explicit Node(ros::NodeHandle * node) : node_(node) {}
  template <class T> T declare_parameter(const std::string & name) const
  {
    T value{};
    // ROS2 parameter paths commonly use dots while ROS1 graph resource names
    // reject them.  Preserve the hierarchy by translating dots to ROS1
    // namespace separators before querying the staged ROS1 parameter tree.
    std::string ros1_name{name};
    std::replace(ros1_name.begin(), ros1_name.end(), '.', '/');
    if (!node_ || !node_->getParam(ros1_name, value)) {
      ROS_WARN_STREAM("[dynamic_obstacle_avoidance] missing parameter: " << name);
    }
    return value;
  }
  Logger get_logger() const { return Logger("dynamic_obstacle_avoidance"); }
  ros::NodeHandle * handle() const { return node_; }
private:
  ros::NodeHandle * node_{};
};
}  // namespace rclcpp

#ifndef RCL_ROS_TIME
#define RCL_ROS_TIME 1
#endif
#ifdef RCLCPP_DEBUG
#undef RCLCPP_DEBUG
#endif
#ifdef RCLCPP_INFO_EXPRESSION
#undef RCLCPP_INFO_EXPRESSION
#endif
#ifdef RCLCPP_ERROR
#undef RCLCPP_ERROR
#endif
#define RCLCPP_DEBUG(logger, ...) ROS_DEBUG(__VA_ARGS__)
#define RCLCPP_INFO_EXPRESSION(logger, condition, ...) do { if (condition) ROS_INFO(__VA_ARGS__); } while (0)
#define RCLCPP_ERROR(logger, ...) ROS_ERROR(__VA_ARGS__)

namespace autoware_utils
{
using autoware_utils_visualization::append_marker_array;
using autoware_utils_visualization::create_default_marker;
using autoware_utils_visualization::create_marker_color;
using autoware_utils_visualization::create_marker_scale;
}  // namespace autoware_utils
