#include <autoware/planning_factor_interface/planning_factor_interface.hpp>

#include <cstdlib>
#include <iostream>

namespace
{
bool received = false;
autoware_internal_planning_msgs::PlanningFactorArray received_msg;

void on_factors(const autoware_internal_planning_msgs::PlanningFactorArray::ConstPtr & msg)
{
  received_msg = *msg;
  received = true;
}

bool require(bool condition, const char * description)
{
  if (!condition) {
    std::cerr << "runtime oracle failed: " << description << std::endl;
  }
  return condition;
}
}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "planning_factor_interface_runtime_oracle");
  ros::NodeHandle node;
  ros::Subscriber subscriber = node.subscribe(
    "/planning/planning_factors/runtime_oracle", 1, on_factors);
  autoware::planning_factor_interface::PlanningFactorInterface interface(
    &node, "runtime_oracle", true, 0);

  autoware_internal_planning_msgs::SafetyFactorArray safety_factors;
  safety_factors.is_safe = true;
  safety_factors.detail = "oracle";
  geometry_msgs::Pose first_pose;
  first_pose.position.x = 10.0;
  first_pose.orientation.w = 1.0;
  geometry_msgs::Pose second_pose;
  second_pose.position.x = 20.0;
  second_pose.orientation.w = 1.0;

  interface.add(5.0, first_pose, autoware_internal_planning_msgs::PlanningFactor::STOP,
    safety_factors, true, 3.0, 0.5, "single");
  interface.add(6.0, 15.0, first_pose, second_pose,
    autoware_internal_planning_msgs::PlanningFactor::SHIFT_LEFT, safety_factors,
    false, 4.0, 5.0, 0.25, -0.25, "section");

  const std::vector<autoware_internal_planning_msgs::PlanningFactor> pending = interface.get_factors();
  if (!require(pending.size() == 2, "queued factor count") ||
      !require(pending[0].control_points.size() == 1, "single control point count") ||
      !require(pending[0].control_points[0].distance == 5.0F, "single distance") ||
      !require(pending[1].control_points.size() == 2, "section control point count") ||
      !require(pending[1].control_points[1].distance == 15.0F, "section end distance") ||
      !require(!pending[1].is_driving_forward, "driving direction")) {
    return EXIT_FAILURE;
  }

  for (int i = 0; i < 100 && subscriber.getNumPublishers() == 0; ++i) {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  if (!require(subscriber.getNumPublishers() == 1, "publisher registration")) {
    return EXIT_FAILURE;
  }

  interface.publish();
  for (int i = 0; i < 100 && !received; ++i) {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  if (!require(received, "published message receipt") ||
      !require(received_msg.header.frame_id == "map", "published frame") ||
      !require(received_msg.factors.size() == 2, "published factor count") ||
      !require(received_msg.factors[0].module == "runtime_oracle", "published module") ||
      !require(received_msg.factors[1].detail == "section", "published detail") ||
      !require(interface.get_factors().empty(), "publish clears queued factors")) {
    return EXIT_FAILURE;
  }

  std::cout << "planning factor interface runtime oracle passed" << std::endl;
  return EXIT_SUCCESS;
}
