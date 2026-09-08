#pragma once

#include <autoware/motion_velocity_planner_common/velocity_planning_result.hpp>
#include <autoware_planning_msgs/TrajectoryPoint.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <visualization_msgs/MarkerArray.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::motion_velocity_planner
{
struct PlannerData;

// The manager OR-combines the requested optional input transports before
// creating subscribers. This preserves the ROS2 per-module readiness contract.
struct RequiredSubscriptionInfo
{
  bool traffic_signals{false};
  bool predicted_objects{false};
  bool occupancy_grid_map{false};
  bool lanelet_map{false};
  bool no_ground_pointcloud{false};

  void update(const RequiredSubscriptionInfo & required)
  {
    traffic_signals |= required.traffic_signals;
    predicted_objects |= required.predicted_objects;
    occupancy_grid_map |= required.occupancy_grid_map;
    lanelet_map |= required.lanelet_map;
    no_ground_pointcloud |= required.no_ground_pointcloud;
  }
};

using ParameterMap = std::map<std::string, XmlRpc::XmlRpcValue>;

// ROS1 plugin boundary. NodeHandle and ros::Publisher replace rclcpp objects;
// each invocation still receives immutable planner state and returns an owned
// velocity result, so modules cannot mutate each other's input trajectory.
class PluginModuleInterface
{
public:
  virtual ~PluginModuleInterface() = default;
  virtual void init(ros::NodeHandle & node, const std::string & module_name) = 0;
  virtual RequiredSubscriptionInfo getRequiredSubscriptions() const = 0;
  virtual void updateParameters(const ParameterMap & parameters) = 0;
  virtual VelocityPlanningResult plan(
    const std::vector<autoware_planning_msgs::TrajectoryPoint> & raw_trajectory_points,
    const std::vector<autoware_planning_msgs::TrajectoryPoint> & smoothed_trajectory_points,
    const std::shared_ptr<const PlannerData> & planner_data) = 0;
  virtual std::string getModuleName() const = 0;
  virtual std::string getShortModuleName() const { return getModuleName(); }
  virtual void publishPlanningFactor() {}

protected:
  ros::Publisher debug_publisher_;
  ros::Publisher virtual_wall_publisher_;
};
}  // namespace autoware::motion_velocity_planner
