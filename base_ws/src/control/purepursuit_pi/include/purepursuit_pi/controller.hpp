#pragma once

#include <fstream>
#include <mutex>
#include <string>

#include <autoware_planning_msgs/Trajectory.h>
#include <geometry_msgs/TransformStamped.h>
#include <morai_msgs/CtrlCmd.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include "purepursuit_pi/trajectory_cache.hpp"
#include "purepursuit_pi/vehicle_model.hpp"

namespace purepursuit_pi {

struct EgoState {
  bool valid{false}; ros::Time stamp; double x{0.0}, y{0.0}, z{0.0}, yaw{0.0}; double long_vel{0.0}, lat_vel{0.0};
};
struct LateralConfig { double lookahead{5.0}, steer_limit{0.35}, steer_rate_limit{0.7}; };
struct LongitudinalConfig { double kp{1.0}, ki{0.3}, integral_limit{4.0}, deadband{0.05}, accel_gain{0.12}, brake_gain{0.16}, accel_limit{0.55}, brake_limit{0.65}; bool use_ff{false}; };

double computePurePursuit(const EgoState& ego, const RefPoint& target, const VehicleModel& vehicle, double lookahead);
double computePI(double error, double target_acceleration, double dt, const LongitudinalConfig& config, double* integral);

class Controller {
 public:
  Controller();
 private:
  void trajectoryCallback(const autoware_planning_msgs::Trajectory::ConstPtr& message);
  void moraiCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& message);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& message);
  void controlTick(const ros::TimerEvent& event);
  bool updatePose(const ros::Time& stamp, EgoState* state) const;
  void publishSafeStop(const std::string& reason, const ros::Time& stamp);
  void publishMarker(const EgoState& ego, const Projection& projection, const RefPoint& target, const ros::Time& stamp);
  void publishEgoState(const EgoState& ego);
  void publishRunConfig();
  std::string runLabel() const;
  double numberParam(const std::string& key, double fallback) const;
  bool boolParam(const std::string& key, bool fallback) const;
  std::string stringParam(const std::string& key, const std::string& fallback) const;

  ros::NodeHandle nh_, pnh_;
  tf2_ros::Buffer tf_buffer_; tf2_ros::TransformListener tf_listener_;
  ros::Subscriber trajectory_sub_, ego_sub_, odom_sub_;
  ros::Publisher command_pub_, run_config_pub_, ego_state_pub_, lookahead_pub_, elapsed_pub_, deadline_pub_, state_pub_, marker_pub_;
  ros::Timer timer_;
  mutable std::mutex mutex_;
  TrajectoryCache trajectory_; EgoState ego_;
  VehicleModel vehicle_; LateralConfig lateral_; LongitudinalConfig longitudinal_;
  std::string source_, map_frame_{"map"}, base_frame_{"base_link"}, config_path_, vehicle_name_;
  double control_rate_hz_{20.0}, trajectory_timeout_{2.0}, ego_timeout_{0.3}, safe_stop_brake_{0.2};
  double integral_{0.0}, last_steer_{0.0}; ros::Time last_tick_; std::string last_state_{"WAIT_FOR_INPUT"};
};

}  // namespace purepursuit_pi
