#pragma once

#include <autoware/path_generator/route_path_generator_core.hpp>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace autoware::path_generator
{
class RoutePathGeneratorNode
{
public:
  explicit RoutePathGeneratorNode(ros::NodeHandle node);

private:
  void onMap(const autoware_map_msgs::LaneletMapBin::ConstPtr & message);
  void onRoute(const autoware_planning_msgs::LaneletRoute::ConstPtr & message);
  void onOdometry(const nav_msgs::Odometry::ConstPtr & message);
  void publishIfReady();

  ros::NodeHandle node_;
  ros::Subscriber map_subscriber_;
  ros::Subscriber route_subscriber_;
  ros::Subscriber odometry_subscriber_;
  ros::Publisher path_publisher_;
  ros::Timer planning_timer_;
  autoware_map_msgs::LaneletMapBin::ConstPtr map_;
  autoware_planning_msgs::LaneletRoute::ConstPtr route_;
  geometry_msgs::Pose initial_pose_{};
  geometry_msgs::Pose current_pose_{};
  bool has_initial_pose_{false};
  bool has_current_pose_{false};
  RoutePathGeneratorParameters parameters_{};
};
}  // namespace autoware::path_generator
