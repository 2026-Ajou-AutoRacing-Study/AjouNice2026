#pragma once

#include <autoware_internal_planning_msgs/VelocityLimit.h>
#include <autoware_perception_msgs/PredictedObjects.h>
#include <autoware_perception_msgs/TrafficLightGroup.h>
#include <autoware_perception_msgs/TrafficLightGroupArray.h>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/node_handle.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <lanelet2_core/primitives/Primitive.h>

#include <deque>
#include <map>
#include <memory>
#include <optional>

namespace autoware::velocity_smoother
{
class SmootherBase;
}

namespace autoware::route_handler
{
class RouteHandler;
}

namespace autoware::behavior_velocity_planner
{
struct TrafficSignalStamped
{
  ros::Time stamp;
  autoware_perception_msgs::TrafficLightGroup signal;
};

// Staged ROS1 planner state. Concrete route-handler and smoother
// implementations remain pointer-owned so this public data ABI does not pull
// their heavy runtime headers into every plugin consumer.
struct PlannerData
{
  PlannerData() = default;
  explicit PlannerData(const ros::NodeHandle & node);

  std::shared_ptr<const geometry_msgs::PoseStamped> current_odometry;
  std::shared_ptr<const geometry_msgs::TwistStamped> current_velocity;
  std::shared_ptr<const geometry_msgs::AccelWithCovarianceStamped> current_acceleration;
  static constexpr double velocity_buffer_time_sec = 10.0;
  std::deque<geometry_msgs::TwistStamped> velocity_buffer;
  std::shared_ptr<const autoware_perception_msgs::PredictedObjects> predicted_objects;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr no_ground_pointcloud;
  std::shared_ptr<const nav_msgs::OccupancyGrid> occupancy_grid;
  double ego_nearest_dist_threshold{3.0};
  double ego_nearest_yaw_threshold{1.57};
  std::optional<autoware_internal_planning_msgs::VelocityLimit> external_velocity_limit;
  std::map<lanelet::Id, TrafficSignalStamped> traffic_light_id_map_raw_;
  std::map<lanelet::Id, TrafficSignalStamped> traffic_light_id_map_last_observed_;
  bool is_simulation{false};
  std::shared_ptr<autoware::velocity_smoother::SmootherBase> velocity_smoother_;
  std::shared_ptr<autoware::route_handler::RouteHandler> route_handler_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

  double max_stop_acceleration_threshold{0.0};
  double max_stop_jerk_threshold{0.0};
  double system_delay{0.0};
  double delay_response_time{0.0};

  bool isVehicleStopped(double stop_duration = 0.0) const;
  std::optional<TrafficSignalStamped> getTrafficSignal(
    lanelet::Id id, bool keep_last_observation = false) const;
  void updateOdometry(const nav_msgs::Odometry & message);
  void updateTrafficSignals(const autoware_perception_msgs::TrafficLightGroupArray & message);
};

// This is the public cross-plugin subscription contract.  Keep it in the
// common ABI even while the full scene-module runtime is staged separately:
// the planner manager combines these requirements before wiring subscribers.
struct RequiredSubscriptionInfo
{
  bool traffic_signals{false};
  bool predicted_objects{false};
  bool occupancy_grid_map{false};
  bool no_ground_pointcloud{false};

  void concat(const RequiredSubscriptionInfo & required_subscriptions)
  {
    traffic_signals |= required_subscriptions.traffic_signals;
    predicted_objects |= required_subscriptions.predicted_objects;
    occupancy_grid_map |= required_subscriptions.occupancy_grid_map;
    no_ground_pointcloud |= required_subscriptions.no_ground_pointcloud;
  }
};
}  // namespace autoware::behavior_velocity_planner
