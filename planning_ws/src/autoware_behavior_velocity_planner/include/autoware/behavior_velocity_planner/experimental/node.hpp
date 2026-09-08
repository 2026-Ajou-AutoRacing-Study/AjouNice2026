#pragma once

#include "autoware/behavior_velocity_planner/experimental/planner_manager.hpp"

#include <autoware/behavior_velocity_planner_common/planner_data.hpp>
#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <autoware_internal_planning_msgs/VelocityLimit.h>
#include <autoware_map_msgs/LaneletMapBin.h>
#include <autoware_perception_msgs/PredictedObjects.h>
#include <autoware_perception_msgs/TrafficLightGroupArray.h>
#include <autoware_planning_msgs/Path.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <mutex>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{
// ROS1 transport boundary for the default upstream BVP node family.  It keeps
// the experimental trajectory/plugin ABI separate from the legacy node.
class BehaviorVelocityPlannerNode
{
public:
  explicit BehaviorVelocityPlannerNode(const ros::NodeHandle & node);

private:
  void onPath(const autoware_internal_planning_msgs::PathWithLaneId::ConstPtr & message);
  void onOdometry(const nav_msgs::Odometry::ConstPtr & message);
  void onMap(const autoware_map_msgs::LaneletMapBin::ConstPtr & message);
  void onAcceleration(const geometry_msgs::AccelWithCovarianceStamped::ConstPtr & message);
  void onObjects(const autoware_perception_msgs::PredictedObjects::ConstPtr & message);
  void onOccupancy(const nav_msgs::OccupancyGrid::ConstPtr & message);
  void onTrafficSignals(const autoware_perception_msgs::TrafficLightGroupArray::ConstPtr & message);
  void onVelocityLimit(const autoware_internal_planning_msgs::VelocityLimit::ConstPtr & message);
  void onNoGroundPointCloud(const sensor_msgs::PointCloud2::ConstPtr & message);
  bool isDataReady() const;

  ros::NodeHandle private_node_{"~"};
  ros::Subscriber path_sub_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber map_sub_;
  ros::Subscriber acceleration_sub_;
  ros::Subscriber objects_sub_;
  ros::Subscriber occupancy_sub_;
  ros::Subscriber traffic_sub_;
  ros::Subscriber velocity_limit_sub_;
  ros::Subscriber no_ground_pointcloud_sub_;
  ros::Publisher path_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  PlannerData planner_data_;
  BehaviorVelocityPlannerManager planner_manager_;
  std::mutex mutex_;
};
}  // namespace autoware::behavior_velocity_planner::experimental
