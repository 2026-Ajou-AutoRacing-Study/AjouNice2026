#include "boundary_departure_prevention_module.hpp"

#include <autoware_adapi_v1_msgs/OperationModeState.h>
#include <autoware_control_msgs/Control.h>
#include <autoware_planning_msgs/LaneletRoute.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <autoware_vehicle_msgs/SteeringReport.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace autoware::motion_velocity_planner::experimental
{
class BoundaryDepartureInputGateOracle
{
public:
  static int run()
  {
    ros::NodeHandle node;
    BoundaryDeparturePreventionModule module;
    module.init(node, "boundary_departure_input_gate_oracle");

    const auto raw = std::vector<autoware_planning_msgs::TrajectoryPoint>(4);
    if (!module.is_data_invalid(raw).has_value()) {
      return fail("BDP_INPUT_GATE_MISSING_DATA_DID_NOT_FAIL_CLOSED");
    }

    auto predicted_pub = node.advertise<autoware_planning_msgs::Trajectory>(
      "/control/trajectory_follower/lateral/predicted_trajectory", 1);
    auto control_pub = node.advertise<autoware_control_msgs::Control>(
      "/control/command/control_cmd", 1);
    auto steering_pub = node.advertise<autoware_vehicle_msgs::SteeringReport>(
      "/vehicle/status/steering_status", 1);
    auto mode_pub = node.advertise<autoware_adapi_v1_msgs::OperationModeState>(
      "/api/operation_mode/state", 1);
    auto route_pub = node.advertise<autoware_planning_msgs::LaneletRoute>(
      "/planning/mission_planning/route", 1);

    if (!wait_until([&] {
          return predicted_pub.getNumSubscribers() == 1 && control_pub.getNumSubscribers() == 1 &&
                 steering_pub.getNumSubscribers() == 1 && mode_pub.getNumSubscribers() == 1 &&
                 route_pub.getNumSubscribers() == 1;
        })) {
      return fail("BDP_INPUT_GATE_FIVE_SUBSCRIBERS_NOT_REGISTERED");
    }

    const auto stamp = ros::Time::now();
    autoware_planning_msgs::Trajectory predicted;
    predicted.header.stamp = stamp;
    predicted.points.emplace_back();
    autoware_control_msgs::Control control;
    control.stamp.sec = stamp.sec;
    control.stamp.nanosec = stamp.nsec;
    autoware_vehicle_msgs::SteeringReport steering;
    // This provider retains the ROS2 builtin_interfaces time layout under
    // its ROS1-facing message name; set the wire fields explicitly.
    steering.stamp.sec = stamp.sec;
    steering.stamp.nanosec = stamp.nsec;
    autoware_adapi_v1_msgs::OperationModeState autonomous;
    autonomous.stamp.sec = stamp.sec;
    autonomous.stamp.nanosec = stamp.nsec;
    autonomous.mode = autoware_adapi_v1_msgs::OperationModeState::AUTONOMOUS;
    autonomous.is_autoware_control_enabled = true;
    autoware_planning_msgs::LaneletRoute route;
    route.header.stamp = stamp;

    predicted_pub.publish(predicted);
    control_pub.publish(control);
    steering_pub.publish(steering);
    mode_pub.publish(autonomous);
    route_pub.publish(route);

    if (!wait_until([&] {
          return module.ego_pred_traj_ptr_ && module.control_cmd_ptr_ && module.steering_angle_ptr_ &&
                 module.op_mode_state_ptr_ && module.route_ptr_;
        })) {
      return fail("BDP_INPUT_GATE_CALLBACK_DELIVERY_FAILED");
    }
    if (module.is_data_invalid(raw).has_value()) {
      return fail("BDP_INPUT_GATE_COMPLETE_FRESH_SET_REJECTED");
    }
    if (!module.is_autonomous_mode()) {
      return fail("BDP_INPUT_GATE_AUTONOMOUS_MODE_REJECTED");
    }

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    if (module.is_data_timeout(odom).has_value()) {
      return fail("BDP_INPUT_GATE_FRESH_SET_REPORTED_TIMEOUT");
    }

    ros::WallDuration(1.15).sleep();
    odom.header.stamp = ros::Time::now();
    if (!module.is_data_timeout(odom).has_value()) {
      return fail("BDP_INPUT_GATE_STALE_SET_DID_NOT_FAIL_CLOSED");
    }

    autonomous.mode = autoware_adapi_v1_msgs::OperationModeState::STOP;
    mode_pub.publish(autonomous);
    if (!wait_until([&] {
          return module.op_mode_state_ptr_ &&
                 module.op_mode_state_ptr_->mode == autoware_adapi_v1_msgs::OperationModeState::STOP;
        })) {
      return fail("BDP_INPUT_GATE_NON_AUTONOMOUS_CALLBACK_FAILED");
    }
    if (module.is_autonomous_mode()) {
      return fail("BDP_INPUT_GATE_NON_AUTONOMOUS_DID_NOT_FAIL_CLOSED");
    }

    std::cout << "BDP_INPUT_GATE_FIVE_TOPIC_CALLBACK_FRESH_STALE_FAIL_CLOSED_PASS" << std::endl;
    return 0;
  }

private:
  static bool wait_until(const std::function<bool()> & predicate)
  {
    const auto deadline = ros::WallTime::now() + ros::WallDuration(5.0);
    while (ros::ok() && ros::WallTime::now() < deadline) {
      ros::spinOnce();
      if (predicate()) {
        return true;
      }
      ros::WallDuration(0.02).sleep();
    }
    return false;
  }

  static int fail(const std::string & reason)
  {
    std::cerr << reason << std::endl;
    return 2;
  }
};
}  // namespace autoware::motion_velocity_planner::experimental

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "bdp_input_gate_oracle");
  return autoware::motion_velocity_planner::experimental::BoundaryDepartureInputGateOracle::run();
}
