#pragma once

#include "autoware/behavior_velocity_planner/planner_manager.hpp"

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
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner
{
// ROS1 runtime foundation for the planner's trigger/manager/output path.
// The data-provider and scene-module gates are added only when their concrete
// dependencies are staged; this node intentionally does not claim them here.
class BehaviorVelocityPlannerNode
{
public:
  explicit BehaviorVelocityPlannerNode(const ros::NodeHandle & node);

private:
  void onTrigger(const autoware_internal_planning_msgs::PathWithLaneId::ConstPtr & input_path);
  void onOdometry(const nav_msgs::Odometry::ConstPtr & odometry);
  void onAcceleration(const geometry_msgs::AccelWithCovarianceStamped::ConstPtr & acceleration);
  void onPredictedObjects(const autoware_perception_msgs::PredictedObjects::ConstPtr & objects);
  void onOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr & occupancy_grid);
  void onExternalVelocityLimit(
    const autoware_internal_planning_msgs::VelocityLimit::ConstPtr & velocity_limit);
  void onTrafficSignals(
    const autoware_perception_msgs::TrafficLightGroupArray::ConstPtr & traffic_signals);
  void onLaneletMap(const autoware_map_msgs::LaneletMapBin::ConstPtr & map);
  void onNoGroundPointCloud(const sensor_msgs::PointCloud2::ConstPtr & pointcloud);

  ros::NodeHandle node_;
  ros::NodeHandle private_node_{"~"};
  ros::Subscriber trigger_sub_path_with_lane_id_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber acceleration_sub_;
  ros::Subscriber predicted_objects_sub_;
  ros::Subscriber occupancy_grid_sub_;
  ros::Subscriber external_velocity_limit_sub_;
  ros::Subscriber traffic_signals_sub_;
  ros::Subscriber lanelet_map_sub_;
  ros::Subscriber no_ground_pointcloud_sub_;
  ros::Publisher path_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  PlannerData planner_data_;
  BehaviorVelocityPlannerManager planner_manager_;
  std::mutex mutex_;
};
}  // namespace autoware::behavior_velocity_planner
