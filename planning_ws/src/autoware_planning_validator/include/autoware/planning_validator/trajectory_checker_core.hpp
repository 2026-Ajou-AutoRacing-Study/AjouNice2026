#ifndef AUTOWARE_PLANNING_VALIDATOR_TRAJECTORY_CHECKER_CORE_HPP_
#define AUTOWARE_PLANNING_VALIDATOR_TRAJECTORY_CHECKER_CORE_HPP_

#include <autoware_planning_msgs/Trajectory.h>
#include <autoware_planning_msgs/LaneletRoute.h>
#include <autoware_map_msgs/LaneletMapBin.h>
#include <autoware_perception_msgs/TrafficLightGroupArray.h>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <autoware_planning_validator/PlanningValidatorStatus.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>

namespace autoware::route_handler
{
class RouteHandler;
}

namespace autoware::planning_validator
{
// Mirrors upstream trajectory_checker.param.yaml defaults for checks rendered
// by R002. Checks not yet backed by a source-equivalent provider stay false.
struct TrajectoryCheckerCoreParameters
{
  double interval_threshold{100.0};
  double curvature_threshold{1.0};
  double relative_angle_threshold{2.0};
  double lateral_accel_threshold{9.8};
  double min_lon_accel_threshold{-9.8};
  double max_lon_accel_threshold{9.8};
  double lateral_jerk_threshold{7.0};
  double steering_threshold{1.414};
  double steering_rate_threshold{10.0};
  double velocity_deviation_threshold{100.0};
  double distance_deviation_threshold{100.0};
  double longitudinal_distance_deviation_threshold{1.0};
  double yaw_deviation_threshold{1.5708};
  double trajectory_yaw_shift_threshold{0.1};
  double wheel_base_m{2.79};
};

struct TrajectoryCheckerInput
{
  const autoware_planning_msgs::Trajectory & trajectory;
  const nav_msgs::Odometry * current_kinematics{nullptr};
  // Preserve the upstream validator's source inputs in the ROS1 plugin ABI.
  // A checker must fail closed when its source-required provider is absent;
  // no adapter may manufacture any of these messages.
  const sensor_msgs::PointCloud2 * obstacle_pointcloud{nullptr};
  const geometry_msgs::AccelWithCovarianceStamped * current_acceleration{nullptr};
  const autoware_planning_msgs::LaneletRoute * route{nullptr};
  const autoware_map_msgs::LaneletMapBin * lanelet_map{nullptr};
  const autoware_perception_msgs::TrafficLightGroupArray * traffic_signals{nullptr};
  const autoware_planning_msgs::Trajectory * resampled_trajectory{nullptr};
  const autoware::vehicle_info_utils::VehicleInfo * vehicle_info{nullptr};
  tf2_ros::Buffer * tf_buffer{nullptr};
  // Collision-checker plugins use the source RouteHandler routing graph and
  // centerline APIs. Passing only the received map/route messages would force
  // each plugin to fabricate or reinitialize routing state, so the node-owned
  // ready handler is an explicit non-owning input instead.
  const autoware::route_handler::RouteHandler * route_handler{nullptr};
  // Appended to preserve the ABI offsets consumed by already-built external
  // providers. It is owned by the node and is never synthesized.
  const autoware_planning_msgs::Trajectory * last_valid_trajectory{nullptr};
  bool vehicle_stopped{false};
};

class TrajectoryCheckerCore
{
public:
  explicit TrajectoryCheckerCore(TrajectoryCheckerCoreParameters parameters = {});
  autoware_planning_validator::PlanningValidatorStatus validate(
    const autoware_planning_msgs::Trajectory & trajectory) const;
  autoware_planning_validator::PlanningValidatorStatus validate(
    const TrajectoryCheckerInput & input) const;

private:
  TrajectoryCheckerCoreParameters parameters_;
};
}  // namespace autoware::planning_validator

#endif
