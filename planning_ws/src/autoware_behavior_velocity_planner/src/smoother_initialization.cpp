#include "smoother_initialization.hpp"
#include <autoware/velocity_smoother/ros1_compat.hpp>
#include <autoware/velocity_smoother/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.hpp>
namespace autoware::behavior_velocity_planner {
namespace {
struct OwnedSmoother {
  // The ROS1 parameter/publisher adapter outlives its smoother, including any
  // retained publisher context. No borrowed stack-node lifetime is assumed.
  rclcpp::Node adapter;
  autoware::velocity_smoother::AnalyticalJerkConstrainedSmoother smoother;
  OwnedSmoother(const ros::NodeHandle & node, double wheel_base)
  : adapter(node), smoother(adapter) { smoother.setWheelBase(wheel_base); }
};
}
std::shared_ptr<autoware::velocity_smoother::SmootherBase> makeBvpVelocitySmoother(
  const ros::NodeHandle & private_node, double wheel_base)
{
  auto owner = std::make_shared<OwnedSmoother>(private_node, wheel_base);
  return {owner, &owner->smoother};
}
}
