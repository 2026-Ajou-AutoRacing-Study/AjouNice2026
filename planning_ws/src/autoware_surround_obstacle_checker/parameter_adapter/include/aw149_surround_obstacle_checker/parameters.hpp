#pragma once

#include <ros/ros.h>

#include <array>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace aw149::surround_obstacle_checker
{
struct ObstacleTypeParameters
{
  bool enable_check{};
  double surround_check_front_distance{};
  double surround_check_side_distance{};
  double surround_check_back_distance{};
};

struct Parameters
{
  std::map<std::string, ObstacleTypeParameters> obstacle_types;
  double surround_check_hysteresis_distance{};
  double state_clear_time{};
  double stop_state_ego_speed{};
  bool publish_debug_footprints{};
  std::string debug_footprint_label;

  template <class T>
  static T require(const ros::NodeHandle & node, const std::string & key)
  {
    T value{};
    if (!node.getParam(key, value)) {
      throw std::runtime_error("missing required ROS1 surround-obstacle parameter: " + key);
    }
    return value;
  }

  static Parameters init(const ros::NodeHandle & node)
  {
    Parameters result;
    const std::array<std::string, 9> labels{
      "pointcloud", "unknown", "car", "truck", "bus", "trailer", "motorcycle", "bicycle", "pedestrian"};
    for (const auto & label : labels) {
      const auto root = label + "/";
      result.obstacle_types.emplace(label, ObstacleTypeParameters{
        require<bool>(node, root + "enable_check"),
        require<double>(node, root + "surround_check_front_distance"),
        require<double>(node, root + "surround_check_side_distance"),
        require<double>(node, root + "surround_check_back_distance")});
    }
    result.surround_check_hysteresis_distance = require<double>(node, "surround_check_hysteresis_distance");
    result.state_clear_time = require<double>(node, "state_clear_time");
    result.stop_state_ego_speed = require<double>(node, "stop_state_ego_speed");
    result.publish_debug_footprints = require<bool>(node, "publish_debug_footprints");
    result.debug_footprint_label = require<std::string>(node, "debug_footprint_label");
    if (result.obstacle_types.count(result.debug_footprint_label) != 1U) {
      throw std::runtime_error("debug_footprint_label is not an obstacle type");
    }
    return result;
  }
};
}  // namespace aw149::surround_obstacle_checker
