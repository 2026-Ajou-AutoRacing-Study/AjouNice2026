#pragma once

#include <autoware_planning_msgs/Trajectory.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <functional>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace autoware::planning_validator::test_utils {
using Trajectory = autoware_planning_msgs::Trajectory;
using AccelWithCovarianceStamped = geometry_msgs::AccelWithCovarianceStamped;
using Odometry = nav_msgs::Odometry;

Trajectory generateTrajectoryWithConstantAcceleration(double interval_distance, double speed, double yaw, size_t size, double acceleration);
Trajectory generateTrajectoryWithVariableAcceleration(double interval_distance, double initial_speed, double yaw, size_t size, const std::function<double(size_t)> & acceleration_pattern);
Trajectory generateTrajectoryWithSinusoidalAcceleration(double interval_distance, double initial_speed, double yaw, size_t size, double max_acceleration, double oscillation_period);
Trajectory generateTrajectoryWithStepAcceleration(double interval_distance, double initial_speed, double yaw, size_t size, const std::vector<double> & acceleration_values, size_t steps_per_value);
Trajectory generateTrajectory(double interval_distance, double speed = 1.0, double yaw = 0.0, size_t size = 10);
Trajectory generateTrajectoryWithConstantCurvature(double interval_distance, double speed, double curvature, size_t size, double wheelbase);
Trajectory generateTrajectoryWithConstantSteering(double interval_distance, double speed, double steering_angle_rad, size_t size, double wheelbase);
Trajectory generateTrajectoryWithConstantSteeringRate(double interval_distance, double speed, double steering_rate, size_t size, double wheelbase);
Trajectory generateShiftedTrajectory(const Trajectory & trajectory, double lat_shift = 0.0, double lon_shift = 0.0, size_t size = std::numeric_limits<size_t>::max());
Trajectory generateNanTrajectory();
Trajectory generateInfTrajectory();
Trajectory generateBadCurvatureTrajectory();
Odometry generateDefaultOdometry(double x = 0.0, double y = 0.0, double vx = 0.0);
AccelWithCovarianceStamped generateDefaultAcceleration(double ax = 0.0);

// ROS1 has no ROS2 node-options parameter-overrides object. This exact
// parameter set is applied to a NodeHandle before constructing the validator.
struct DefaultParameters {
  std::map<std::string, double> doubles;
  std::map<std::string, int> integers;
  std::map<std::string, bool> booleans;
  void apply(const ros::NodeHandle & node) const;
};
DefaultParameters getDefaultParameters();
}  // namespace autoware::planning_validator::test_utils
