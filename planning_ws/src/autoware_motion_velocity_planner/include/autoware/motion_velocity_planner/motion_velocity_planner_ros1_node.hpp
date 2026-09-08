#pragma once

#include <autoware/motion_velocity_planner/motion_velocity_planner_core.hpp>
#include <autoware/motion_velocity_planner/motion_velocity_planner_manager.hpp>
#include <autoware/motion_velocity_planner_common/planner_data.hpp>
#include <autoware_internal_planning_msgs/VelocityLimit.h>
#include <autoware_internal_planning_msgs/VelocityLimitClearCommand.h>
#include <autoware_perception_msgs/PredictedObjects.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <autoware_map_msgs/LaneletMapBin.h>

namespace autoware::motion_velocity_planner
{
// ROS1 transport around the staged ordered-result composition core.  It has no
// module manager: receiving a trajectory composes ordered module results and
// transports trajectory plus VelocityLimit/clear-limit outputs. Plugin
// loading, map/object readiness, smoothing, and debug transport remain
// explicitly scoped candidate responsibilities.
class MotionVelocityPlannerRos1Node
{
public:
  explicit MotionVelocityPlannerRos1Node(ros::NodeHandle private_node);
  size_t loadedModuleCount() const { return manager_.size(); }
  bool hasLaunchModuleConfiguration() const { return has_launch_module_configuration_; }

private:
  void onTrajectory(const autoware_planning_msgs::TrajectoryConstPtr & message);
  void onPredictedObjects(const autoware_perception_msgs::PredictedObjectsConstPtr & message);
  void onNoGroundPointcloud(const sensor_msgs::PointCloud2ConstPtr & message);
  void onOccupancyGridMap(const nav_msgs::OccupancyGridConstPtr & message);
  void onLaneletMap(const autoware_map_msgs::LaneletMapBinConstPtr & message);
  void onOdometry(const nav_msgs::OdometryConstPtr & message);
  void onAcceleration(const geometry_msgs::AccelWithCovarianceStampedConstPtr & message);

  ros::NodeHandle private_node_;
  ros::Subscriber trajectory_subscriber_;
  ros::Subscriber predicted_objects_subscriber_;
  ros::Subscriber no_ground_pointcloud_subscriber_;
  ros::Subscriber occupancy_grid_map_subscriber_;
  ros::Subscriber lanelet_map_subscriber_;
  ros::Subscriber odometry_subscriber_;
  ros::Subscriber acceleration_subscriber_;
  ros::Publisher trajectory_publisher_;
  ros::Publisher velocity_limit_publisher_;
  ros::Publisher clear_velocity_limit_publisher_;
  MotionVelocityPlannerCore core_;
  MotionVelocityPlannerManager manager_;
  std::shared_ptr<PlannerData> planner_data_{std::make_shared<PlannerData>()};
  bool has_launch_module_configuration_{false};
  bool dynamic_obstacle_stop_loaded_{false};
  bool velocity_smoother_ready_{true};
  bool predicted_objects_received_{false};
  bool no_ground_pointcloud_received_{false};
  bool occupancy_grid_map_received_{false};
  bool lanelet_map_received_{false};
  bool odometry_received_{false};
  bool acceleration_received_{false};
};
}  // namespace autoware::motion_velocity_planner
