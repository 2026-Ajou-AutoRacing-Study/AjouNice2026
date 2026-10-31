#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <autoware_planning_msgs/Trajectory.h>
#include <morai_msgs/CtrlCmd.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <trajectory_follower/TerminalState.h>
#include <trajectory_follower/distance_speed_limit.hpp>
#include <trajectory_follower/longitudinal_pi_utils.hpp>

namespace
{
constexpr const char * kAnsiYellow = "\033[1;33m";
constexpr const char * kAnsiRed = "\033[1;31m";
constexpr const char * kAnsiReset = "\033[0m";

std::string normalizedFrame(const std::string & frame)
{
  const std::size_t first = frame.find_first_not_of('/');
  return first == std::string::npos ? std::string() : frame.substr(first);
}

bool finiteState(const nav_msgs::Odometry & state)
{
  const auto & pose = state.pose.pose;
  const auto & linear = state.twist.twist.linear;
  const double quaternion_norm = pose.orientation.x * pose.orientation.x + pose.orientation.y * pose.orientation.y +
    pose.orientation.z * pose.orientation.z + pose.orientation.w * pose.orientation.w;
  return !normalizedFrame(state.header.frame_id).empty() && std::isfinite(pose.position.x) &&
    std::isfinite(pose.position.y) && std::isfinite(pose.position.z) && std::isfinite(quaternion_norm) &&
    quaternion_norm > 1e-12 && std::isfinite(linear.x) && std::isfinite(linear.y) && std::isfinite(linear.z);
}

bool sourceStampFresh(
  const ros::Time & stamp, const ros::Time & now, const double timeout_s,
  const double future_tolerance_s)
{
  if (stamp.isZero()) {
    return false;
  }
  const double age_s = (now - stamp).toSec();
  return std::isfinite(age_s) && age_s <= timeout_s && age_s >= -future_tolerance_s;
}
}  // namespace

// Keeps the existing MORAI type-1 PI longitudinal interface, but treats lateral-controller
// validity as a safety input. It intentionally does not port Autoware's longitudinal PID.
class AutowareLongitudinalPi
{
public:
  AutowareLongitudinalPi() : private_nh_("~")
  {
    private_nh_.param<std::string>("topics/trajectory", trajectory_topic_, "/control/trajectory");
    private_nh_.param<std::string>("topics/kinematic_state", state_topic_, "/localization/kinematic_state");
    private_nh_.param<std::string>("topics/steering_cmd", steering_topic_, "/control/command/steering");
    private_nh_.param<std::string>("topics/controller_valid", valid_topic_,
                                   "/control/autoware/controller_valid");
    private_nh_.param<std::string>("topics/failure_reason", failure_topic_,
                                   "/control/autoware/failure_reason");
    private_nh_.param<std::string>("topics/ctrl_cmd", ctrl_cmd_topic_, "/ctrl_cmd");
    private_nh_.param("control_rate_hz", control_rate_hz_, 50.0);
    private_nh_.param("logging/status_period_s", status_period_s_, 1.0);
    private_nh_.param("kinematic_velocity_scale_to_mps", velocity_scale_, 1.0);
    if (!private_nh_.getParam("vehicle/max_steer_rad", max_steer_rad_)) {
      double max_wheel_angle_deg = 40.0;
      private_nh_.param("vehicle/max_wheel_angle_deg", max_wheel_angle_deg, 40.0);
      max_steer_rad_ = max_wheel_angle_deg * M_PI / 180.0;
    }
    private_nh_.param("control_limits/max_accel_cmd", max_accel_cmd_, 1.0);
    private_nh_.param("control_limits/max_brake_cmd", max_brake_cmd_, 1.0);
    private_nh_.param(
      "control_limits/max_brake_rise_rate_per_s", max_brake_rise_rate_per_s_, 2.0);
    private_nh_.param(
      "control_limits/max_brake_fall_rate_per_s", max_brake_fall_rate_per_s_, 5.0);
    private_nh_.param("safety/trajectory_timeout_s", trajectory_timeout_s_, 0.5);
    private_nh_.param("safety/state_timeout_s", state_timeout_s_, 0.5);
    private_nh_.param("safety/trajectory_source_timeout_s", trajectory_source_timeout_s_, 0.5);
    private_nh_.param("safety/state_source_timeout_s", state_source_timeout_s_, 0.5);
    private_nh_.param("safety/future_stamp_tolerance_s", future_stamp_tolerance_s_, 0.1);
    private_nh_.param("safety/steering_timeout_s", steering_timeout_s_, 0.5);
    private_nh_.param("safety/lateral_valid_timeout_s", lateral_valid_timeout_s_, 0.2);
    private_nh_.param("safety/min_points", min_points_, 3);
    private_nh_.param("safety/stop_brake_cmd", stop_brake_cmd_, 1.0);
    private_nh_.param("safety/pose_jump/enabled", pose_jump_enabled_, true);
    private_nh_.param<std::string>("safety/pose_jump/state_topic", pose_jump_topic_,
      "/localization/kinematic_state");
    private_nh_.param("safety/pose_jump/distance_m", pose_jump_distance_m_, 7.0);
    private_nh_.param("safety/pose_jump/brake_duration_s", pose_jump_brake_duration_s_, 1.0);
    private_nh_.param("safety/pose_jump/motion_tolerance_m", pose_jump_motion_tolerance_m_, 1.0);
    private_nh_.param("safety/pose_jump/max_acceleration_mps2", pose_jump_max_acceleration_mps2_, 10.0);
    private_nh_.param("kp", kp_, 0.6);
    private_nh_.param("ki", ki_, 0.05);
    private_nh_.param("integral_limit", integral_limit_, 3.0);
    private_nh_.param("command_deadband", command_deadband_, 0.05);
    private_nh_.param("acceleration_feedforward/enabled", acceleration_feedforward_enabled_, false);
    private_nh_.param("acceleration_feedforward/max_acceleration_mps2", feedforward_max_acceleration_, 3.7);
    private_nh_.param("acceleration_feedforward/gain", feedforward_gain_, 1.0);
    private_nh_.param("acceleration_feedforward/max_rise_rate_per_s", feedforward_rise_rate_, 0.0);
    XmlRpc::XmlRpcValue acceleration_map;
    if (private_nh_.getParam("acceleration_feedforward/speed_acceleration_map", acceleration_map)) {
      const auto number = [](XmlRpc::XmlRpcValue & value) -> double {
        if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {return static_cast<double>(value);}
        if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {return static_cast<int>(value);}
        throw std::runtime_error("FF speed_acceleration_map requires numeric [m/s, m/s^2] pairs");
      };
      if (acceleration_map.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        throw std::runtime_error("FF speed_acceleration_map must be an array");
      }
      for (int i = 0; i < acceleration_map.size(); ++i) {
        auto & point = acceleration_map[i];
        if (point.getType() != XmlRpc::XmlRpcValue::TypeArray || point.size() != 2) {
          throw std::runtime_error("FF speed_acceleration_map requires [m/s, m/s^2] pairs");
        }
        feedforward_acceleration_map_.emplace_back(number(point[0]), number(point[1]));
      }
    }
    private_nh_.param(
      "reset_acceleration_integral_on_braking", reset_acceleration_integral_on_braking_, true);
    private_nh_.param("brake_hysteresis/enabled", brake_hysteresis_enabled_, true);
    private_nh_.param("brake_hysteresis/enter_error_mps", brake_enter_error_mps_, 0.08);
    private_nh_.param("brake_hysteresis/exit_error_mps", brake_exit_error_mps_, 0.15);
    private_nh_.param("brake_hysteresis/hard_exit_error_mps", brake_hard_exit_error_mps_, 0.5);
    private_nh_.param("brake_hysteresis/hard_exit_hold_s", brake_hard_exit_hold_s_, 0.1);
    private_nh_.param(
      "brake_hysteresis/planned_deceleration_threshold_mps2",
      brake_planned_deceleration_threshold_mps2_, 0.5);
    private_nh_.param("brake_hysteresis/min_brake_cmd", brake_min_command_, 0.05);
    private_nh_.param("speed_preview/enabled", speed_preview_enabled_, true);
    private_nh_.param("speed_preview/time_s", speed_preview_time_s_, 1.5);
    private_nh_.param("speed_preview/min_distance_m", speed_preview_min_distance_m_, 5.0);
    private_nh_.param("speed_preview/max_distance_m", speed_preview_max_distance_m_, 30.0);
    private_nh_.param("speed_preview/deceleration_mps2", preview_deceleration_mps2_, 2.0);
    private_nh_.param("speed_preview/response_delay_s", preview_response_delay_s_, 0.3);
    private_nh_.param("speed_preview/distance_margin_m", preview_distance_margin_m_, 0.2);
    private_nh_.param("stop/velocity_threshold_mps", stop_velocity_threshold_mps_, 0.1);
    private_nh_.param("stop/hold_brake_cmd", hold_brake_cmd_, 0.2);
    private_nh_.param("stop/smooth_stop_enabled", smooth_stop_enabled_, false);
    private_nh_.param("stop/smooth_stop_entry_velocity_mps", smooth_stop_entry_velocity_mps_, 0.5);
    private_nh_.param("stop/smooth_stop_max_brake_fall_rate_per_s",
      smooth_stop_max_brake_fall_rate_per_s_, 0.5);
    private_nh_.param("deceleration_feedback/enabled", deceleration_feedback_enabled_, false);
    private_nh_.param("deceleration_feedback/activation_mps2", deceleration_feedback_activation_mps2_, 0.3);
    private_nh_.param("deceleration_feedback/max_deceleration_mps2",
      deceleration_feedback_max_deceleration_mps2_, 2.0);
    private_nh_.param("deceleration_feedback/safety_max_deceleration_mps2",
      deceleration_feedback_safety_max_deceleration_mps2_, 8.0);
    private_nh_.param("deceleration_feedback/acceleration_lpf_time_constant_s",
      acceleration_lpf_time_constant_s_, 0.2);
    private_nh_.param("deceleration_feedback/brake_integrator_gain",
      deceleration_feedback_brake_integrator_gain_, 0.35);
    private_nh_.param("deceleration_feedback/speed_error_override_mps",
      deceleration_feedback_speed_error_override_mps_, 1.0);
    private_nh_.param("deceleration_feedback/safety_pi_max_brake_cmd",
      deceleration_feedback_safety_pi_max_brake_cmd_, 0.4);
    private_nh_.param("deceleration_feedback/velocity_error_gain",
      deceleration_feedback_velocity_error_gain_, 0.35);
    private_nh_.param("deceleration_feedback/max_velocity_correction_mps2",
      deceleration_feedback_max_velocity_correction_mps2_, 0.4);
    private_nh_.param("deceleration_feedback/overspeed_velocity_error_gain",
      deceleration_feedback_overspeed_velocity_error_gain_, 0.5);
    private_nh_.param("deceleration_feedback/max_overspeed_correction_mps2",
      deceleration_feedback_max_overspeed_correction_mps2_, 0.5);
    validateParameters();

    command_publisher_ = nh_.advertise<morai_msgs::CtrlCmd>(ctrl_cmd_topic_, 10);
    terminal_publisher_ = nh_.advertise<trajectory_follower::TerminalState>("/control/terminal_state", 1);
    debug_publisher_ = private_nh_.advertise<std_msgs::Float64MultiArray>("debug/longitudinal", 10);
    trajectory_subscriber_ = nh_.subscribe(trajectory_topic_, 2,
      &AutowareLongitudinalPi::trajectoryCallback, this);
    state_subscriber_ = nh_.subscribe(state_topic_, 10, &AutowareLongitudinalPi::stateCallback, this);
    if (pose_jump_enabled_) {
      pose_jump_subscriber_ = nh_.subscribe(pose_jump_topic_, 10,
        &AutowareLongitudinalPi::poseJumpCallback, this);
    }
    steering_subscriber_ = nh_.subscribe(steering_topic_, 10, &AutowareLongitudinalPi::steeringCallback, this);
    validity_subscriber_ = nh_.subscribe(valid_topic_, 10, &AutowareLongitudinalPi::validityCallback, this);
    failure_subscriber_ = nh_.subscribe(failure_topic_, 10,
      &AutowareLongitudinalPi::failureCallback, this);
    timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_hz_),
      &AutowareLongitudinalPi::timerCallback, this);
    ROS_INFO_STREAM(
      "[CTRL][LONG][START] trajectory=" << trajectory_topic_ << " | state=" << state_topic_
      << " | command=" << ctrl_cmd_topic_ << " | rate=" << control_rate_hz_ << "Hz | status="
      << status_period_s_ << "s");
  }

private:
  void validateParameters() const
  {
    const bool valid = !pose_jump_topic_.empty() &&
      std::isfinite(pose_jump_distance_m_) && pose_jump_distance_m_ > 0.0 &&
      std::isfinite(pose_jump_brake_duration_s_) && pose_jump_brake_duration_s_ > 0.0 &&
      std::isfinite(pose_jump_motion_tolerance_m_) && pose_jump_motion_tolerance_m_ >= 0.0 &&
      std::isfinite(pose_jump_max_acceleration_mps2_) && pose_jump_max_acceleration_mps2_ >= 0.0 &&
      std::isfinite(control_rate_hz_) && control_rate_hz_ > 0.0 &&
      std::isfinite(status_period_s_) && status_period_s_ > 0.0 &&
      std::isfinite(velocity_scale_) && velocity_scale_ > 0.0 && std::isfinite(max_steer_rad_) &&
      max_steer_rad_ > 0.0 && std::isfinite(max_accel_cmd_) && max_accel_cmd_ > 0.0 &&
      max_accel_cmd_ <= 1.0 && std::isfinite(max_brake_cmd_) && max_brake_cmd_ > 0.0 &&
      max_brake_cmd_ <= 1.0 && std::isfinite(stop_brake_cmd_) && stop_brake_cmd_ >= 0.0 &&
      std::isfinite(max_brake_rise_rate_per_s_) && max_brake_rise_rate_per_s_ > 0.0 &&
      std::isfinite(max_brake_fall_rate_per_s_) && max_brake_fall_rate_per_s_ > 0.0 &&
      stop_brake_cmd_ <= 1.0 && std::isfinite(trajectory_timeout_s_) && trajectory_timeout_s_ > 0.0 &&
      std::isfinite(state_timeout_s_) && state_timeout_s_ > 0.0 && std::isfinite(steering_timeout_s_) &&
      steering_timeout_s_ > 0.0 && std::isfinite(lateral_valid_timeout_s_) && lateral_valid_timeout_s_ > 0.0 &&
      std::isfinite(trajectory_source_timeout_s_) && trajectory_source_timeout_s_ > 0.0 &&
      std::isfinite(state_source_timeout_s_) && state_source_timeout_s_ > 0.0 &&
      std::isfinite(future_stamp_tolerance_s_) && future_stamp_tolerance_s_ >= 0.0 &&
      min_points_ >= 3 && std::isfinite(kp_) && kp_ >= 0.0 && std::isfinite(ki_) && ki_ >= 0.0 &&
      std::isfinite(integral_limit_) && integral_limit_ > 0.0 && std::isfinite(command_deadband_) &&
      command_deadband_ >= 0.0 &&
      std::isfinite(feedforward_max_acceleration_) && feedforward_max_acceleration_ > 0.0 &&
      std::isfinite(feedforward_gain_) && feedforward_gain_ >= 0.0 &&
      std::isfinite(feedforward_rise_rate_) && feedforward_rise_rate_ >= 0.0 &&
      trajectory_follower::validAccelerationMap(feedforward_acceleration_map_) &&
      std::isfinite(brake_enter_error_mps_) && brake_enter_error_mps_ >= 0.0 &&
      std::isfinite(brake_exit_error_mps_) && brake_exit_error_mps_ > brake_enter_error_mps_ &&
      std::isfinite(brake_hard_exit_error_mps_) && brake_hard_exit_error_mps_ >= brake_exit_error_mps_ &&
      std::isfinite(brake_hard_exit_hold_s_) && brake_hard_exit_hold_s_ >= 0.0 &&
      std::isfinite(brake_planned_deceleration_threshold_mps2_) &&
      brake_planned_deceleration_threshold_mps2_ > 0.0 && std::isfinite(brake_min_command_) &&
      brake_min_command_ >= 0.0 && brake_min_command_ <= max_brake_cmd_ &&
      std::isfinite(speed_preview_time_s_) && speed_preview_time_s_ >= 0.0 &&
      std::isfinite(speed_preview_min_distance_m_) && speed_preview_min_distance_m_ >= 0.0 &&
      std::isfinite(speed_preview_max_distance_m_) &&
      speed_preview_max_distance_m_ >= speed_preview_min_distance_m_ &&
      std::isfinite(preview_deceleration_mps2_) && preview_deceleration_mps2_ > 0.0 &&
      std::isfinite(preview_response_delay_s_) && preview_response_delay_s_ >= 0.0 &&
      std::isfinite(preview_distance_margin_m_) && preview_distance_margin_m_ >= 0.0 &&
      std::isfinite(stop_velocity_threshold_mps_) && stop_velocity_threshold_mps_ >= 0.0 &&
      std::isfinite(hold_brake_cmd_) && hold_brake_cmd_ >= 0.0 && hold_brake_cmd_ <= max_brake_cmd_ &&
      std::isfinite(smooth_stop_entry_velocity_mps_) &&
      smooth_stop_entry_velocity_mps_ > stop_velocity_threshold_mps_ &&
      std::isfinite(smooth_stop_max_brake_fall_rate_per_s_) &&
      smooth_stop_max_brake_fall_rate_per_s_ > 0.0 &&
      std::isfinite(deceleration_feedback_activation_mps2_) &&
      deceleration_feedback_activation_mps2_ > 0.0 &&
      std::isfinite(deceleration_feedback_max_deceleration_mps2_) &&
      deceleration_feedback_max_deceleration_mps2_ > 0.0 &&
      std::isfinite(deceleration_feedback_safety_max_deceleration_mps2_) &&
      deceleration_feedback_safety_max_deceleration_mps2_ >= deceleration_feedback_max_deceleration_mps2_ &&
      std::isfinite(acceleration_lpf_time_constant_s_) && acceleration_lpf_time_constant_s_ >= 0.0 &&
      std::isfinite(deceleration_feedback_brake_integrator_gain_) &&
      deceleration_feedback_brake_integrator_gain_ >= 0.0 &&
      std::isfinite(deceleration_feedback_speed_error_override_mps_) &&
      deceleration_feedback_speed_error_override_mps_ > 0.0 &&
      std::isfinite(deceleration_feedback_safety_pi_max_brake_cmd_) &&
      deceleration_feedback_safety_pi_max_brake_cmd_ >= 0.0 &&
      deceleration_feedback_safety_pi_max_brake_cmd_ <= max_brake_cmd_ &&
      std::isfinite(deceleration_feedback_velocity_error_gain_) &&
      deceleration_feedback_velocity_error_gain_ >= 0.0 &&
      std::isfinite(deceleration_feedback_max_velocity_correction_mps2_) &&
      deceleration_feedback_max_velocity_correction_mps2_ >= 0.0 &&
      std::isfinite(deceleration_feedback_overspeed_velocity_error_gain_) &&
      deceleration_feedback_overspeed_velocity_error_gain_ >= 0.0 &&
      std::isfinite(deceleration_feedback_max_overspeed_correction_mps2_) &&
      deceleration_feedback_max_overspeed_correction_mps2_ >= 0.0;
    if (!valid) {
      throw std::runtime_error("invalid longi_pi_autoware parameter");
    }
  }

  void trajectoryCallback(const autoware_planning_msgs::Trajectory::ConstPtr & message)
  {
    bool valid = !normalizedFrame(message->header.frame_id).empty() &&
      message->points.size() >= static_cast<std::size_t>(min_points_);
    for (const auto & point : message->points) {
      valid = valid && std::isfinite(point.pose.position.x) && std::isfinite(point.pose.position.y) &&
        std::isfinite(point.longitudinal_velocity_mps) && std::isfinite(point.acceleration_mps2);
    }
    trajectory_valid_ = valid;
    if (valid) {
      trajectory_ = *message;
      trajectory_received_ = ros::Time::now();
    }
  }

  void stateCallback(const nav_msgs::Odometry::ConstPtr & message)
  {
    state_valid_ = finiteState(*message);
    if (state_valid_) {
      state_ = *message;
      state_received_ = ros::Time::now();
    }
  }

  void poseJumpCallback(const nav_msgs::Odometry::ConstPtr & message)
  {
    const ros::Time now = ros::Time::now();
    if (!finiteState(*message) || !sourceStampFresh(
        message->header.stamp, now, state_source_timeout_s_, future_stamp_tolerance_s_)) {
      return;
    }
    const bool same_frame = pose_jump_initialized_ &&
      normalizedFrame(message->header.frame_id) == normalizedFrame(pose_jump_previous_.header.frame_id);
    // Duplicate/out-of-order packets must not turn an old pose into a new jump.
    if (same_frame && message->header.stamp <= pose_jump_previous_.header.stamp) {
      return;
    }
    if (same_frame && sourceStampFresh(
        pose_jump_previous_.header.stamp, now, state_source_timeout_s_, future_stamp_tolerance_s_)) {
      const double distance = std::hypot(
        message->pose.pose.position.x - pose_jump_previous_.pose.pose.position.x,
        message->pose.pose.position.y - pose_jump_previous_.pose.pose.position.y);
      const double dt = (message->header.stamp - pose_jump_previous_.header.stamp).toSec();
      const auto speed = [](const nav_msgs::Odometry & state) {
          const auto & v = state.twist.twist.linear;
          return std::hypot(std::hypot(v.x, v.y), v.z);
        };
      // Global planning odometry is in m/s. A brief receive gap can span >7 m
      // during ordinary driving, so distance alone is not a teleport detector.
      const double normal_motion_bound =
        std::max(speed(*message), speed(pose_jump_previous_)) * dt +
        0.5 * pose_jump_max_acceleration_mps2_ * dt * dt + pose_jump_motion_tolerance_m_;
      if (distance >= pose_jump_distance_m_ && distance > normal_motion_bound) {
        // Use monotonic elapsed time; a ROS clock adjustment cannot shorten the hold.
        pose_jump_brake_until_ = ros::SteadyTime::now() + ros::WallDuration(pose_jump_brake_duration_s_);
        ROS_WARN("Pose jump %.3f m in %.3f s (normal bound %.3f m) on %s: braking for %.3f s",
          distance, dt, normal_motion_bound, pose_jump_topic_.c_str(), pose_jump_brake_duration_s_);
        publishStop("WARN", "pose jump braking");
      }
    }
    pose_jump_previous_ = *message;
    pose_jump_initialized_ = true;
  }

  void steeringCallback(const std_msgs::Float64::ConstPtr & message)
  {
    steering_valid_ = std::isfinite(message->data);
    if (steering_valid_) {
      steering_ = std::max(-max_steer_rad_, std::min(max_steer_rad_, message->data));
      steering_received_ = ros::Time::now();
    }
  }

  void validityCallback(const std_msgs::Bool::ConstPtr & message)
  {
    lateral_valid_ = message->data;
    lateral_valid_received_ = ros::Time::now();
  }

  void failureCallback(const std_msgs::String::ConstPtr & message)
  {
    lateral_failure_reason_ = message->data;
  }

  static void appendReason(std::vector<std::string> & reasons, const std::string & reason)
  {
    if (!reason.empty()) {
      reasons.push_back(reason);
    }
  }

  std::string inputFailureReason(const ros::Time & now) const
  {
    std::vector<std::string> reasons;
    const auto receipt_reason = [&](const bool valid, const ros::Time & received,
                                    const double timeout, const char * name) {
      if (!valid || received.isZero()) {
        appendReason(reasons, std::string(name) + " unavailable");
        return;
      }
      const double age = (now - received).toSec();
      if (!std::isfinite(age) || age > timeout) {
        std::ostringstream text;
        text.setf(std::ios::fixed);
        text.precision(2);
        text << name << " rx stale " << age << "s";
        appendReason(reasons, text.str());
      }
    };
    receipt_reason(trajectory_valid_, trajectory_received_, trajectory_timeout_s_, "trajectory");
    receipt_reason(state_valid_, state_received_, state_timeout_s_, "state");
    receipt_reason(steering_valid_, steering_received_, steering_timeout_s_, "steering");
    receipt_reason(lateral_valid_, lateral_valid_received_, lateral_valid_timeout_s_, "lateral");
    if (trajectory_valid_ && !sourceStampFresh(
          trajectory_.header.stamp, now, trajectory_source_timeout_s_, future_stamp_tolerance_s_)) {
      appendReason(reasons, "trajectory source stamp stale/zero/future");
    }
    if (state_valid_ && !sourceStampFresh(
          state_.header.stamp, now, state_source_timeout_s_, future_stamp_tolerance_s_)) {
      appendReason(reasons, "state source stamp stale/zero/future");
    }
    if (!lateral_valid_ && !lateral_failure_reason_.empty()) {
      appendReason(reasons, "lateral: " + lateral_failure_reason_);
    }
    if (reasons.empty()) {
      return "controller input not ready";
    }
    std::ostringstream joined;
    for (std::size_t i = 0; i < reasons.size(); ++i) {
      if (i != 0U) {
        joined << "; ";
      }
      joined << reasons[i];
    }
    return joined.str();
  }

  void logSafetyStop(const std::string & reason, const bool force_error = false) const
  {
    if (force_error || ever_ready_) {
      ROS_ERROR_THROTTLE(
        status_period_s_, "%s[CTRL][LONG][SAFE_STOP] %s | accel=0.00 brake=%.2f steer=0deg%s",
        kAnsiRed, reason.c_str(), stop_brake_cmd_, kAnsiReset);
    } else {
      ROS_WARN_THROTTLE(
        status_period_s_, "%s[CTRL][LONG][WAIT] %s | accel=0.00 brake=%.2f steer=0deg%s",
        kAnsiYellow, reason.c_str(), stop_brake_cmd_, kAnsiReset);
    }
  }

  std::size_t nearestIndex() const
  {
    std::size_t nearest = 0;
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < trajectory_.points.size(); ++i) {
      const double dx = trajectory_.points[i].pose.position.x - state_.pose.pose.position.x;
      const double dy = trajectory_.points[i].pose.position.y - state_.pose.pose.position.y;
      const double distance_squared = dx * dx + dy * dy;
      if (distance_squared < best) {
        best = distance_squared;
        nearest = i;
      }
    }
    return nearest;
  }

  double accelerationTargetVelocity(const std::size_t nearest, const double fallback) const
  {
    if (!acceleration_feedforward_enabled_ || speed_preview_enabled_ ||
        trajectory_.points[nearest].acceleration_mps2 <= 0.0 ||
        fallback <= stop_velocity_threshold_mps_) {
      return fallback;
    }
    double best_distance = std::numeric_limits<double>::infinity();
    double result = fallback;
    for (std::size_t i = nearest > 0 ? nearest - 1 : 0;
         i <= nearest && i + 1 < trajectory_.points.size(); ++i) {
      const auto & first = trajectory_.points[i];
      const auto & second = trajectory_.points[i + 1];
      const double dx = second.pose.position.x - first.pose.position.x;
      const double dy = second.pose.position.y - first.pose.position.y;
      const double length_squared = dx * dx + dy * dy;
      if (length_squared <= std::numeric_limits<double>::epsilon()) {
        continue;
      }
      const double ex = state_.pose.pose.position.x - first.pose.position.x;
      const double ey = state_.pose.pose.position.y - first.pose.position.y;
      const double ratio = std::max(0.0, std::min(1.0, (ex * dx + ey * dy) / length_squared));
      const double px = ex - ratio * dx;
      const double py = ey - ratio * dy;
      const double distance_squared = px * px + py * py;
      if (distance_squared >= best_distance) {
        continue;
      }
      best_distance = distance_squared;
      result = fallback;
      // Preserve all deceleration/stop targets, including a transition segment.
      if (first.acceleration_mps2 > 0.0 && second.acceleration_mps2 > 0.0 &&
          first.longitudinal_velocity_mps > stop_velocity_threshold_mps_) {
        result = trajectory_follower::interpolateIncreasingVelocity(
          first.longitudinal_velocity_mps, second.longitudinal_velocity_mps, ratio, fallback);
      }
    }
    return result;
  }

  double previewTargetVelocity(const std::size_t nearest, const double current_velocity) const
  {
    const double initial_target = trajectory_.points[nearest].longitudinal_velocity_mps;
    // Treat small negative velocities inside the stop threshold as standstill noise.
    // Only a velocity below -0.1 m/s with the production parameter is considered reverse.
    if (!speed_preview_enabled_ || initial_target < 0.0 ||
        current_velocity < -stop_velocity_threshold_mps_) {
      return initial_target;
    }

    const double approach_speed = std::max(std::abs(current_velocity), initial_target);
    const double preview_distance = std::max(trajectory_follower::brakingPreviewDistance(
      approach_speed, preview_deceleration_mps2_, preview_response_delay_s_,
      preview_distance_margin_m_), std::max(
      speed_preview_min_distance_m_, std::min(
        speed_preview_max_distance_m_, std::abs(current_velocity) * speed_preview_time_s_)));
    double target_velocity = initial_target;
    // Account for ego lying between sampled points; nearest-point distance alone
    // overestimates the stopping distance when ego is ahead of that point.
    double traversed_distance = 0.0;
    if (nearest + 1U < trajectory_.points.size()) {
      const auto & p = trajectory_.points[nearest].pose.position;
      const auto & q = trajectory_.points[nearest + 1U].pose.position;
      const double dx = q.x - p.x, dy = q.y - p.y;
      const double length = std::hypot(dx, dy);
      if (length > 1e-6) {
        const auto & ego = state_.pose.pose.position;
        traversed_distance = -std::max(-length, std::min(length,
          ((ego.x - p.x) * dx + (ego.y - p.y) * dy) / length));
      }
    }
    for (std::size_t i = nearest; i + 1U < trajectory_.points.size(); ++i) {
      const auto & first = trajectory_.points[i];
      const auto & second = trajectory_.points[i + 1U];
      traversed_distance += std::hypot(
        second.pose.position.x - first.pose.position.x,
        second.pose.position.y - first.pose.position.y);
      // Preserve the current planned speed unless a future restriction requires
      // braking now. A stop five metres ahead is not a zero-speed command here.
      if (second.longitudinal_velocity_mps >= 0.0) {
        target_velocity = std::min(target_velocity, trajectory_follower::distanceSpeedLimit(
          second.longitudinal_velocity_mps, traversed_distance, approach_speed,
          preview_deceleration_mps2_, preview_response_delay_s_, preview_distance_margin_m_));
      }
      // Include the first point beyond the horizon to cover sparse segments.
      if (traversed_distance >= preview_distance) break;
    }
    return target_velocity;
  }

  // Observation only: mirrors the final command and existing safety branch.
  void publishCommand(const morai_msgs::CtrlCmd & command,
                      const std::string & level = "OK", const std::string & reason = "")
  {
    command_publisher_.publish(command);
    trajectory_follower::TerminalState status;
    status.header.stamp = ros::Time::now();
    status.level = level;
    status.reason = reason;
    status.steering_rad = command.steering;
    terminal_publisher_.publish(status);
  }

  void publishHold(const double steering)
  {
    last_feedforward_ = 0.0;
    integral_ = 0.0;
    braking_mode_ = false;
    brake_hard_exit_elapsed_s_ = 0.0;
    last_brake_cmd_ = hold_brake_cmd_;
    acceleration_initialized_ = false;
    morai_msgs::CtrlCmd command;
    command.longlCmdType = 1;
    command.accel = 0.0;
    command.brake = hold_brake_cmd_;
    command.steering = steering;
    command.velocity = 0.0;
    command.acceleration = 0.0;
    publishCommand(command);
  }

  void publishStop(const std::string & level = "OK", const std::string & reason = "")
  {
    last_feedforward_ = 0.0;
    integral_ = 0.0;
    braking_mode_ = false;
    brake_hard_exit_elapsed_s_ = 0.0;
    last_brake_cmd_ = stop_brake_cmd_;
    acceleration_initialized_ = false;
    morai_msgs::CtrlCmd command;
    command.longlCmdType = 1;
    command.accel = 0.0;
    command.brake = stop_brake_cmd_;
    command.steering = 0.0;
    command.velocity = 0.0;
    command.acceleration = 0.0;
    publishCommand(command, level, reason);
  }

  void timerCallback(const ros::TimerEvent & event)
  {
    if (pose_jump_enabled_ && ros::SteadyTime::now() < pose_jump_brake_until_) {
      publishStop("WARN", "pose jump braking");
      return;
    }
    const ros::Time now = ros::Time::now();
    const bool fresh = trajectory_valid_ && state_valid_ && steering_valid_ && lateral_valid_ &&
      (now - trajectory_received_).toSec() <= trajectory_timeout_s_ &&
      (now - state_received_).toSec() <= state_timeout_s_ &&
      (now - steering_received_).toSec() <= steering_timeout_s_ &&
      (now - lateral_valid_received_).toSec() <= lateral_valid_timeout_s_ &&
      sourceStampFresh(
        trajectory_.header.stamp, now, trajectory_source_timeout_s_, future_stamp_tolerance_s_) &&
      sourceStampFresh(state_.header.stamp, now, state_source_timeout_s_, future_stamp_tolerance_s_);
    if (!fresh || normalizedFrame(trajectory_.header.frame_id) != normalizedFrame(state_.header.frame_id)) {
      const bool frame_mismatch = fresh &&
        normalizedFrame(trajectory_.header.frame_id) != normalizedFrame(state_.header.frame_id);
      const std::string reason = frame_mismatch ? "trajectory/state frame mismatch" : inputFailureReason(now);
      logSafetyStop(reason, frame_mismatch);
      publishStop((frame_mismatch || ever_ready_) ? "ERROR" : "WAIT", reason);
      return;
    }
    ever_ready_ = true;
    const std::size_t nearest = nearestIndex();
    const double current_velocity = state_.twist.twist.linear.x * velocity_scale_;
    if (nearest + 1U >= trajectory_.points.size()) {
      ROS_INFO_THROTTLE(
        status_period_s_,
        "[CTRL][LONG][PATH_END] v=%.2fm/s (%.1fkm/h) | accel=0.00 brake=%.2f | path=%zu/%zu",
        current_velocity, current_velocity * 3.6, stop_brake_cmd_, nearest + 1U,
        trajectory_.points.size());
      publishStop();
      return;
    }
    const double target_velocity = accelerationTargetVelocity(
      nearest, previewTargetVelocity(nearest, current_velocity));
    if (target_velocity <= stop_velocity_threshold_mps_ &&
        std::abs(current_velocity) <= stop_velocity_threshold_mps_) {
      ROS_INFO_THROTTLE(
        status_period_s_,
        "[CTRL][LONG][HOLD] v=%.2f->%.2fm/s (%.1f->%.1fkm/h) | accel=0.00 brake=%.2f | "
        "steer=%+.2fdeg | path=%zu/%zu | age(T/S)=%.2f/%.2fs",
        current_velocity, target_velocity, current_velocity * 3.6, target_velocity * 3.6,
        hold_brake_cmd_, steering_ * 180.0 / M_PI, nearest + 1U, trajectory_.points.size(),
        (now - trajectory_received_).toSec(), (now - state_received_).toSec());
      publishHold(steering_);
      return;
    }
    const double error = target_velocity - current_velocity;
    const double planned_acceleration = trajectory_.points[nearest].acceleration_mps2;
    const bool smooth_stop_active = smooth_stop_enabled_ &&
      target_velocity <= stop_velocity_threshold_mps_ &&
      std::abs(current_velocity) <= smooth_stop_entry_velocity_mps_;
    double dt = (event.current_real - event.last_real).toSec();
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 1.0) {
      dt = 1.0 / control_rate_hz_;
    }
    if (acceleration_initialized_) {
      const double raw_acceleration = (current_velocity - previous_velocity_mps_) / dt;
      if (std::isfinite(raw_acceleration)) {
        const double alpha = acceleration_lpf_time_constant_s_ <= 0.0
          ? 1.0 : dt / (acceleration_lpf_time_constant_s_ + dt);
        filtered_acceleration_mps2_ += std::max(0.0, std::min(1.0, alpha)) *
          (raw_acceleration - filtered_acceleration_mps2_);
      }
    } else {
      filtered_acceleration_mps2_ = 0.0;
      acceleration_initialized_ = true;
    }
    previous_velocity_mps_ = current_velocity;
    const bool deceleration_feedback_active = deceleration_feedback_enabled_ &&
      !smooth_stop_active && planned_acceleration <= -deceleration_feedback_activation_mps2_ &&
      acceleration_initialized_;
    if (deceleration_feedback_active) {
      // Avoid carrying a velocity-PI windup into acceleration-controlled braking.
      integral_ = 0.0;
    }
    integral_ = trajectory_follower::resetAccelerationIntegralOnBraking(
      error, integral_, reset_acceleration_integral_on_braking_);
    const auto brake_hysteresis = trajectory_follower::updateBrakeHysteresis(
      {braking_mode_, brake_hard_exit_elapsed_s_}, error, planned_acceleration, dt,
      brake_hysteresis_enabled_,
      brake_enter_error_mps_, brake_exit_error_mps_, brake_hard_exit_error_mps_,
      brake_hard_exit_hold_s_,
      brake_planned_deceleration_threshold_mps2_);
    braking_mode_ = brake_hysteresis.active;
    brake_hard_exit_elapsed_s_ = brake_hysteresis.hard_exit_elapsed_s;
    const bool acceleration_allowed = !braking_mode_ && !smooth_stop_active &&
      !deceleration_feedback_active && last_brake_cmd_ <= command_deadband_ &&
      current_velocity >= 0.0 && target_velocity > stop_velocity_threshold_mps_;
    const double feedforward_weight = trajectory_follower::accelerationFeedforwardWeight(
      error, brake_hysteresis_enabled_ ? brake_enter_error_mps_ : 0.0);
    const double calibrated_acceleration = trajectory_follower::accelerationAtSpeed(
      feedforward_acceleration_map_, current_velocity, feedforward_max_acceleration_);
    const double requested_feedforward = trajectory_follower::accelerationFeedforward(
      acceleration_feedforward_enabled_, acceleration_allowed, planned_acceleration,
      calibrated_acceleration, feedforward_gain_) * feedforward_weight;
    const double feedforward = trajectory_follower::limitFeedforwardRise(
      requested_feedforward, last_feedforward_, feedforward_rise_rate_, dt);
    const double candidate_integral = std::max(-integral_limit_, std::min(integral_limit_, integral_ + error * dt));
    const double candidate_command = kp_ * error + ki_ * candidate_integral;
    if (!((candidate_command + feedforward > max_accel_cmd_ && error > 0.0) ||
          (candidate_command < -max_brake_cmd_ && error < 0.0))) {
      integral_ = candidate_integral;
    }
    const double command_value = kp_ * error + ki_ * integral_;
    morai_msgs::CtrlCmd command;
    command.longlCmdType = 1;
    command.steering = steering_;
    command.velocity = 0.0;
    command.acceleration = 0.0;
    // Only during permitted positive acceleration, PI trims the feedforward
    // before choosing a pedal. Braking mode, nonpositive planned acceleration,
    // hold and watchdog retain the original brake arbitration.
    const double pedal_command = command_value + feedforward;
    double desired_brake = pedal_command < -command_deadband_
      ? std::min(max_brake_cmd_, -pedal_command) : 0.0;
    if (deceleration_feedback_active) {
      const bool safety_override = error < -deceleration_feedback_speed_error_override_mps_;
      const double maximum_deceleration = safety_override
        ? deceleration_feedback_safety_max_deceleration_mps2_
        : deceleration_feedback_max_deceleration_mps2_;
      double desired_acceleration = std::max(-maximum_deceleration,
        std::min(0.0, planned_acceleration));
      if (!safety_override) {
        // A bounded outer velocity loop closes the residual gap to the raw target
        // without converting the raw trajectory acceleration directly to pedal.
        const bool overspeed = error < 0.0;
        const double correction_gain = overspeed
          ? deceleration_feedback_overspeed_velocity_error_gain_
          : deceleration_feedback_velocity_error_gain_;
        const double correction_limit = overspeed
          ? deceleration_feedback_max_overspeed_correction_mps2_
          : deceleration_feedback_max_velocity_correction_mps2_;
        const double velocity_correction = std::max(-correction_limit,
          std::min(correction_limit, correction_gain * error));
        desired_acceleration = std::max(-(maximum_deceleration + correction_limit),
          std::min(0.0, desired_acceleration + velocity_correction));
      }
      // If measured deceleration is too weak, increase brake; if it is too strong,
      // release it gradually. This avoids mapping planned acceleration directly to pedal.
      const double acceleration_error = filtered_acceleration_mps2_ - desired_acceleration;
      const double feedback_brake = last_brake_cmd_ +
        deceleration_feedback_brake_integrator_gain_ * acceleration_error * dt;
      desired_brake = std::max(0.0, std::min(max_brake_cmd_, feedback_brake));
      // Retain the existing velocity PI only for a large overspeed safety correction.
      if (safety_override) {
        const double bounded_pi_brake = command_value < 0.0
          ? std::min(deceleration_feedback_safety_pi_max_brake_cmd_, -command_value) : 0.0;
        desired_brake = std::max(desired_brake, bounded_pi_brake);
      }
    }
    const bool retain_brake_for_planned_deceleration = braking_mode_ &&
      planned_acceleration <= -brake_planned_deceleration_threshold_mps2_;
    if (retain_brake_for_planned_deceleration) {
      desired_brake = std::max(desired_brake, brake_min_command_);
    }
    if (smooth_stop_active) {
      // Blend continuously into hold brake and suppress terminal brake chatter.
      const double range = smooth_stop_entry_velocity_mps_ - stop_velocity_threshold_mps_;
      const double progress = std::max(0.0, std::min(1.0,
        (smooth_stop_entry_velocity_mps_ - std::abs(current_velocity)) / range));
      desired_brake = std::max(desired_brake, hold_brake_cmd_ * progress);
    }
    const double brake_fall_rate = smooth_stop_active
      ? std::min(max_brake_fall_rate_per_s_, smooth_stop_max_brake_fall_rate_per_s_)
      : max_brake_fall_rate_per_s_;
    const double brake_lower = std::max(0.0, last_brake_cmd_ - brake_fall_rate * dt);
    const double brake_upper = std::min(max_brake_cmd_, last_brake_cmd_ + max_brake_rise_rate_per_s_ * dt);
    command.brake = std::max(brake_lower, std::min(brake_upper, desired_brake));
    last_brake_cmd_ = command.brake;
    // FF is zero at brake entry and must never bypass a brake-release interlock.
    const double applied_feedforward = command.brake <= command_deadband_ &&
      desired_brake == 0.0 ? feedforward : 0.0;
    last_feedforward_ = applied_feedforward;
    const double acceleration_command = command_value + applied_feedforward;
    // Never request throttle while a previous brake command is being released.
    command.accel = !braking_mode_ && !smooth_stop_active && !deceleration_feedback_active &&
      command.brake <= command_deadband_ &&
      acceleration_command > command_deadband_
      ? std::min(max_accel_cmd_, acceleration_command) : 0.0;
    publishCommand(command);
    const char * mode = smooth_stop_active ? "SMOOTH_STOP" :
      deceleration_feedback_active ? "DECEL" :
      (braking_mode_ || command.brake > command_deadband_) ? "BRAKE" :
      command.accel > command_deadband_ ? "ACCEL" : "COAST";
    ROS_INFO_THROTTLE(
      status_period_s_,
      "[CTRL][LONG][%s] v=%.2f->%.2fm/s (%.1f->%.1fkm/h) | accel=%.2f brake=%.2f | "
      "steer=%+.2fdeg | a(plan/meas)=%+.2f/%+.2fm/s2 | path=%zu/%zu | age(T/S)=%.2f/%.2fs",
      mode, current_velocity, target_velocity, current_velocity * 3.6, target_velocity * 3.6,
      command.accel, command.brake, command.steering * 180.0 / M_PI, planned_acceleration,
      filtered_acceleration_mps2_, nearest + 1U, trajectory_.points.size(),
      (now - trajectory_received_).toSec(), (now - state_received_).toSec());
    std_msgs::Float64MultiArray debug;
    debug.layout.dim.resize(1);
    debug.layout.dim[0].label =
      "stamp_s,target_mps,actual_mps,planned_acc_mps2,measured_acc_mps2,P,I,FF,accel,brake,braking_mode";
    debug.data = {now.toSec(), target_velocity, current_velocity, planned_acceleration,
      filtered_acceleration_mps2_, kp_ * error, ki_ * integral_, applied_feedforward,
      command.accel, command.brake, braking_mode_ ? 1.0 : 0.0};
    debug.layout.dim[0].size = debug.data.size();
    debug.layout.dim[0].stride = debug.data.size();
    debug_publisher_.publish(debug);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber trajectory_subscriber_;
  ros::Subscriber state_subscriber_;
  ros::Subscriber pose_jump_subscriber_;
  ros::Subscriber steering_subscriber_;
  ros::Subscriber validity_subscriber_;
  ros::Subscriber failure_subscriber_;
  ros::Publisher command_publisher_;
  ros::Publisher terminal_publisher_;
  ros::Publisher debug_publisher_;
  bool acceleration_feedforward_enabled_{false};
  double feedforward_max_acceleration_{3.7};
  double feedforward_gain_{1.0};
  double feedforward_rise_rate_{0.0};
  trajectory_follower::AccelerationMap feedforward_acceleration_map_;
  double last_feedforward_{0.0};
  ros::Timer timer_;
  std::string trajectory_topic_;
  std::string state_topic_;
  std::string steering_topic_;
  std::string valid_topic_;
  std::string failure_topic_;
  std::string ctrl_cmd_topic_;
  double control_rate_hz_{50.0};
  double status_period_s_{1.0};
  double velocity_scale_{1.0};
  double max_steer_rad_{0.698};
  double max_accel_cmd_{1.0};
  double max_brake_cmd_{1.0};
  double max_brake_rise_rate_per_s_{2.0};
  double max_brake_fall_rate_per_s_{5.0};
  double trajectory_timeout_s_{0.5};
  double state_timeout_s_{0.5};
  double trajectory_source_timeout_s_{0.5};
  double state_source_timeout_s_{0.5};
  double future_stamp_tolerance_s_{0.1};
  double steering_timeout_s_{0.5};
  double lateral_valid_timeout_s_{0.2};
  double stop_brake_cmd_{1.0};
  bool pose_jump_enabled_{true};
  std::string pose_jump_topic_;
  double pose_jump_distance_m_{7.0};
  double pose_jump_brake_duration_s_{1.0};
  double pose_jump_motion_tolerance_m_{1.0};
  double pose_jump_max_acceleration_mps2_{10.0};
  bool pose_jump_initialized_{false};
  nav_msgs::Odometry pose_jump_previous_;
  ros::SteadyTime pose_jump_brake_until_;
  int min_points_{3};
  double kp_{0.6};
  double ki_{0.05};
  double integral_limit_{3.0};
  double command_deadband_{0.05};
  bool reset_acceleration_integral_on_braking_{true};
  bool brake_hysteresis_enabled_{true};
  double brake_enter_error_mps_{0.08};
  double brake_exit_error_mps_{0.15};
  double brake_hard_exit_error_mps_{0.5};
  double brake_hard_exit_hold_s_{0.1};
  double brake_planned_deceleration_threshold_mps2_{0.5};
  double brake_min_command_{0.05};
  bool speed_preview_enabled_{true};
  double speed_preview_time_s_{1.5};
  double speed_preview_min_distance_m_{5.0};
  double speed_preview_max_distance_m_{30.0};
  double preview_deceleration_mps2_{2.0};
  double preview_response_delay_s_{0.3};
  double preview_distance_margin_m_{0.2};
  double stop_velocity_threshold_mps_{0.1};
  double hold_brake_cmd_{0.2};
  bool smooth_stop_enabled_{false};
  double smooth_stop_entry_velocity_mps_{0.5};
  double smooth_stop_max_brake_fall_rate_per_s_{0.5};
  bool deceleration_feedback_enabled_{false};
  double deceleration_feedback_activation_mps2_{0.3};
  double deceleration_feedback_max_deceleration_mps2_{2.0};
  double deceleration_feedback_safety_max_deceleration_mps2_{8.0};
  double acceleration_lpf_time_constant_s_{0.2};
  double deceleration_feedback_brake_integrator_gain_{0.35};
  double deceleration_feedback_speed_error_override_mps_{1.0};
  double deceleration_feedback_safety_pi_max_brake_cmd_{0.4};
  double deceleration_feedback_velocity_error_gain_{0.35};
  double deceleration_feedback_max_velocity_correction_mps2_{0.4};
  double deceleration_feedback_overspeed_velocity_error_gain_{0.5};
  double deceleration_feedback_max_overspeed_correction_mps2_{0.5};
  double last_brake_cmd_{0.0};
  bool acceleration_initialized_{false};
  double previous_velocity_mps_{0.0};
  double filtered_acceleration_mps2_{0.0};
  double integral_{0.0};
  bool braking_mode_{false};
  double brake_hard_exit_elapsed_s_{0.0};
  double steering_{0.0};
  bool trajectory_valid_{false};
  bool state_valid_{false};
  bool steering_valid_{false};
  bool lateral_valid_{false};
  bool ever_ready_{false};
  std::string lateral_failure_reason_;
  ros::Time trajectory_received_;
  ros::Time state_received_;
  ros::Time steering_received_;
  ros::Time lateral_valid_received_;
  autoware_planning_msgs::Trajectory trajectory_;
  nav_msgs::Odometry state_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "longi_pi_autoware");
  try {
    AutowareLongitudinalPi node;
    ros::spin();
  } catch (const std::exception & error) {
    ROS_FATAL("%s[CTRL][LONG][FATAL] %s%s", kAnsiRed, error.what(), kAnsiReset);
    return 1;
  }
  return 0;
}
