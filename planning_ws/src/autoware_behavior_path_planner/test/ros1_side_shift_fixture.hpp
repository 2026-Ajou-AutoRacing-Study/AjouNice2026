#pragma once

#include <ros/ros.h>

#include <cstdlib>
#include <stdexcept>
#include <string>

namespace aw149::test
{
inline void loadSideShiftFixture(const ros::NodeHandle & node)
{
  const std::string command =
    "rosparam load \"$(rospack find autoware_behavior_path_side_shift_module)"
    "/config/ros1/side_shift.yaml\" \"" + node.getNamespace() + "\"";
  if (std::system(command.c_str()) != 0) {
    throw std::runtime_error(
      "failed to load authoritative side-shift fixture into " + node.getNamespace());
  }
}

inline void loadBvpSmootherFixture(ros::NodeHandle & node)
{
  for (const auto * relative : {
      "/config/ros1/default_velocity_smoother.yaml",
      "/config/ros1/Analytical.yaml"}) {
    const std::string command =
      "rosparam load \"$(rospack find autoware_velocity_smoother)" +
      std::string(relative) + "\" \"" + node.getNamespace() + "\"";
    if (std::system(command.c_str()) != 0) {
      throw std::runtime_error(
        "failed to load authoritative BVP smoother fixture into " + node.getNamespace());
    }
  }
  node.setParam("normal/max_acc", 1.0);
  node.setParam("normal/min_acc", -1.0);
  node.setParam("normal/max_jerk", 1.0);
  node.setParam("normal/min_jerk", -1.0);
}
}  // namespace aw149::test
