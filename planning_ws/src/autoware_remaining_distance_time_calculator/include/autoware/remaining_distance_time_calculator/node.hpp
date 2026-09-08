#pragma once

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_internal_msgs/MissionRemainingDistanceTime.h>
#include <autoware_internal_planning_msgs/Scenario.h>
#include <autoware_internal_planning_msgs/VelocityLimit.h>
#include <autoware_map_msgs/LaneletMapBin.h>
#include <autoware_planning_msgs/LaneletRoute.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace autoware::remaining_distance_time_calculator
{
class RemainingDistanceTimeCalculatorNode
{
public:
  explicit RemainingDistanceTimeCalculatorNode(ros::NodeHandle node);

private:
  void on_map(const autoware_map_msgs::LaneletMapBin::ConstPtr & message);
  void on_route(const autoware_planning_msgs::LaneletRoute::ConstPtr & message);
  void on_odometry(const nav_msgs::Odometry::ConstPtr & message);
  void on_velocity_limit(const autoware_internal_planning_msgs::VelocityLimit::ConstPtr & message);
  void on_scenario(const autoware_internal_planning_msgs::Scenario::ConstPtr & message);
  void on_timer(const ros::TimerEvent &);
  bool calculate_remaining_distance(double * distance) const;
  void publish(double distance, double time) const;

  ros::NodeHandle node_;
  ros::Subscriber map_sub_, route_sub_, odometry_sub_, velocity_limit_sub_, scenario_sub_;
  ros::Publisher output_pub_;
  ros::Timer timer_;
  autoware::route_handler::RouteHandler route_handler_;
  autoware_planning_msgs::LaneletRoute::ConstPtr route_;
  autoware_internal_planning_msgs::Scenario::ConstPtr scenario_;
  geometry_msgs::Pose current_pose_;
  bool map_received_{false};
  bool odometry_received_{false};
  double velocity_limit_{99.99};
};
}  // namespace autoware::remaining_distance_time_calculator
