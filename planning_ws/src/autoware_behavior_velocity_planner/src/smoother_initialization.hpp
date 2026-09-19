#pragma once
#include <ros/node_handle.h>
#include <memory>
namespace autoware::velocity_smoother { class SmootherBase; }
namespace autoware::behavior_velocity_planner {
std::shared_ptr<autoware::velocity_smoother::SmootherBase> makeBvpVelocitySmoother(
  const ros::NodeHandle & private_node, double wheel_base);
}
