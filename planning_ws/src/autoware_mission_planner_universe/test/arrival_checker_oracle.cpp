#include "arrival_checker.hpp"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <iostream>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "arrival_checker_oracle");
  ros::NodeHandle private_node("~");
  private_node.setParam("arrival_check_angle_deg", 45.0);
  private_node.setParam("arrival_check_lateral_distance", 1.0);
  private_node.setParam("arrival_check_longitudinal_undershoot_distance", 1.0);
  private_node.setParam("arrival_check_longitudinal_overshoot_distance", 2.0);
  private_node.setParam("arrival_check_duration", 1.0);

  autoware::mission_planner_universe::ArrivalChecker checker(&private_node);
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.orientation.w = 1.0;
  if (checker.is_arrived(pose)) {
    std::cerr << "arrival without goal" << std::endl;
    return 1;
  }

  autoware_planning_msgs::PoseWithUuidStamped goal;
  goal.header.frame_id = "odom";
  goal.pose.orientation.w = 1.0;
  checker.set_goal(goal);
  if (checker.is_arrived(pose)) {
    std::cerr << "arrival with frame mismatch" << std::endl;
    return 2;
  }
  checker.set_goal();
  if (checker.is_arrived(pose)) {
    std::cerr << "arrival after clear" << std::endl;
    return 3;
  }

  goal.header.frame_id = "map";
  goal.pose.orientation.w = 1.0;
  checker.set_goal(goal);
  ros::NodeHandle node;
  auto odometry_publisher = node.advertise<nav_msgs::Odometry>("/localization/kinematic_state", 1, true);
  const auto subscriber_deadline = ros::WallTime::now() + ros::WallDuration(2.0);
  while (
    ros::ok() && ros::WallTime::now() < subscriber_deadline &&
    odometry_publisher.getNumSubscribers() == 0)
  {
    ros::spinOnce();
    ros::WallDuration(0.02).sleep();
  }
  if (odometry_publisher.getNumSubscribers() == 0) {
    std::cerr << "ArrivalChecker odometry subscription was not established" << std::endl;
    return 4;
  }
  nav_msgs::Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.header.stamp = ros::Time::now();
  odometry.pose.pose.orientation.w = 1.0;
  odometry.twist.twist.linear.x = 0.0;
  odometry_publisher.publish(odometry);
  const auto delivery_deadline = ros::WallTime::now() + ros::WallDuration(1.0);
  while (ros::ok() && ros::WallTime::now() < delivery_deadline) {
    ros::spinOnce();
    ros::WallDuration(0.02).sleep();
    if (!checker.is_arrived(pose)) break;
  }
  if (checker.is_arrived(pose)) {
    std::cerr << "arrival before required stopped duration" << std::endl;
    return 5;
  }
  ros::WallDuration(1.05).sleep();
  ros::spinOnce();
  if (!checker.is_arrived(pose)) {
    std::cerr << "arrival after matching stopped-vehicle history" << std::endl;
    return 6;
  }
  std::cout << "arrival_checker_oracle=PASS stopped-history" << std::endl;
  return 0;
}
