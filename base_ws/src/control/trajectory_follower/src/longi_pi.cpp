#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>

#include <autoware_planning_msgs/Trajectory.h>
#include <morai_msgs/CtrlCmd.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace
{

std::string normalizedFrame(const std::string& frame)
{
  const std::size_t first = frame.find_first_not_of('/');
  return first == std::string::npos ? std::string() : frame.substr(first);
}

bool finiteState(const nav_msgs::Odometry& state)
{
  const auto& position = state.pose.pose.position;
  const auto& orientation = state.pose.pose.orientation;
  const auto& linear = state.twist.twist.linear;
  const double quaternion_norm = orientation.x * orientation.x + orientation.y * orientation.y +
                                 orientation.z * orientation.z + orientation.w * orientation.w;
  return !normalizedFrame(state.header.frame_id).empty() && std::isfinite(position.x) &&
         std::isfinite(position.y) && std::isfinite(position.z) && std::isfinite(orientation.x) &&
         std::isfinite(orientation.y) && std::isfinite(orientation.z) && std::isfinite(orientation.w) &&
         std::isfinite(quaternion_norm) && quaternion_norm > 1e-12 && std::isfinite(linear.x) &&
         std::isfinite(linear.y) && std::isfinite(linear.z);
}

}  // namespace

class LongitudinalPi
{
public:
  LongitudinalPi() : private_nh_("~")
  {
    private_nh_.param<std::string>("topics/trajectory", trajectory_topic_, "/control/trajectory");
    private_nh_.param<std::string>("topics/kinematic_state", state_topic_, "/localization/kinematic_state");
    private_nh_.param<std::string>("topics/steering_cmd", steering_topic_, "/control/command/steering");
    private_nh_.param<std::string>("topics/ctrl_cmd", ctrl_cmd_topic_, "/ctrl_cmd");
    private_nh_.param("control_rate_hz", control_rate_hz_, 30.0);
    private_nh_.param("kinematic_velocity_scale_to_mps", kinematic_velocity_scale_to_mps_, 1.0);
    private_nh_.param("control_limits/max_accel_cmd", max_accel_cmd_, 1.0);
    private_nh_.param("control_limits/max_brake_cmd", max_brake_cmd_, 1.0);
    private_nh_.param("vehicle/max_steer_rad", max_steer_rad_, 0.698);
    private_nh_.param("morai/longlCmdType", longl_cmd_type_, 1);
    private_nh_.param("safety/trajectory_timeout_s", trajectory_timeout_s_, 0.5);
    private_nh_.param("safety/state_timeout_s", state_timeout_s_, 0.5);
    private_nh_.param("safety/steering_timeout_s", steering_timeout_s_, 0.5);
    private_nh_.param("safety/min_points", min_points_, 2);
    private_nh_.param("safety/stop_brake_cmd", stop_brake_cmd_, 1.0);
    private_nh_.param("kp", kp_, 0.6);
    private_nh_.param("ki", ki_, 0.05);
    private_nh_.param("integral_limit", integral_limit_, 3.0);
    private_nh_.param("command_deadband", command_deadband_, 0.05);
    validateParameters();

    command_publisher_ = nh_.advertise<morai_msgs::CtrlCmd>(ctrl_cmd_topic_, 10);
    trajectory_subscriber_ =
        nh_.subscribe(trajectory_topic_, 2, &LongitudinalPi::trajectoryCallback, this);
    state_subscriber_ = nh_.subscribe(state_topic_, 10, &LongitudinalPi::stateCallback, this);
    steering_subscriber_ = nh_.subscribe(steering_topic_, 10, &LongitudinalPi::steeringCallback, this);
    timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_hz_), &LongitudinalPi::timerCallback, this);
    ROS_INFO_STREAM("longi_pi: trajectory=" << trajectory_topic_ << " state=" << state_topic_ << " steering="
                                             << steering_topic_ << " -> " << ctrl_cmd_topic_ << " at "
                                             << control_rate_hz_ << " Hz");
  }

private:
  void validateParameters() const
  {
    if (longl_cmd_type_ != 1)
    {
      throw std::runtime_error("morai/longlCmdType must be 1");
    }
    const bool valid = std::isfinite(control_rate_hz_) && control_rate_hz_ > 0.0 &&
                       std::isfinite(kinematic_velocity_scale_to_mps_) &&
                       kinematic_velocity_scale_to_mps_ > 0.0 &&
                       std::isfinite(max_accel_cmd_) && max_accel_cmd_ > 0.0 && max_accel_cmd_ <= 1.0 &&
                       std::isfinite(max_brake_cmd_) && max_brake_cmd_ > 0.0 && max_brake_cmd_ <= 1.0 &&
                       std::isfinite(max_steer_rad_) && max_steer_rad_ > 0.0 &&
                       std::isfinite(stop_brake_cmd_) && stop_brake_cmd_ >= 0.0 && stop_brake_cmd_ <= 1.0 &&
                       std::isfinite(trajectory_timeout_s_) && trajectory_timeout_s_ > 0.0 &&
                       std::isfinite(state_timeout_s_) && state_timeout_s_ > 0.0 &&
                       std::isfinite(steering_timeout_s_) && steering_timeout_s_ > 0.0 &&
                       min_points_ >= 2 && std::isfinite(kp_) && kp_ >= 0.0 && std::isfinite(ki_) && ki_ >= 0.0 &&
                       std::isfinite(integral_limit_) && integral_limit_ > 0.0 &&
                       std::isfinite(command_deadband_) && command_deadband_ >= 0.0 &&
                       command_deadband_ <= std::max(max_accel_cmd_, max_brake_cmd_);
    if (!valid)
    {
      throw std::runtime_error("invalid longitudinal PI parameter");
    }
  }

  void trajectoryCallback(const autoware_planning_msgs::Trajectory::ConstPtr& message)
  {
    bool valid = !normalizedFrame(message->header.frame_id).empty() &&
                 message->points.size() >= static_cast<std::size_t>(min_points_);
    for (const auto& point : message->points)
    {
      valid = valid && std::isfinite(point.pose.position.x) && std::isfinite(point.pose.position.y) &&
              std::isfinite(point.longitudinal_velocity_mps) && point.longitudinal_velocity_mps >= 0.0F;
    }
    if (!valid)
    {
      trajectory_valid_ = false;
      ROS_WARN_THROTTLE(1.0, "longi_pi rejected invalid trajectory");
      return;
    }
    trajectory_ = *message;
    trajectory_valid_ = true;
    trajectory_received_ = ros::Time::now();
  }

  void stateCallback(const nav_msgs::Odometry::ConstPtr& message)
  {
    if (!finiteState(*message))
    {
      state_valid_ = false;
      ROS_WARN_THROTTLE(1.0, "longi_pi rejected invalid kinematic state");
      return;
    }
    state_ = *message;
    state_valid_ = true;
    state_received_ = ros::Time::now();
  }

  void steeringCallback(const std_msgs::Float64::ConstPtr& message)
  {
    if (!std::isfinite(message->data))
    {
      steering_valid_ = false;
      ROS_WARN_THROTTLE(1.0, "longi_pi rejected non-finite steering command");
      return;
    }
    steering_ = std::max(-max_steer_rad_, std::min(max_steer_rad_, message->data));
    steering_valid_ = true;
    steering_received_ = ros::Time::now();
  }

  std::size_t nearestIndex() const
  {
    std::size_t nearest = 0;
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < trajectory_.points.size(); ++index)
    {
      const double dx = trajectory_.points[index].pose.position.x - state_.pose.pose.position.x;
      const double dy = trajectory_.points[index].pose.position.y - state_.pose.pose.position.y;
      const double squared_distance = dx * dx + dy * dy;
      if (squared_distance < best)
      {
        best = squared_distance;
        nearest = index;
      }
    }
    return nearest;
  }

  void publishStop()
  {
    integral_ = 0.0;
    morai_msgs::CtrlCmd command;
    command.longlCmdType = 1;
    command.accel = 0.0;
    command.brake = stop_brake_cmd_;
    command.steering = 0.0;
    command.velocity = 0.0;
    command.acceleration = 0.0;
    command_publisher_.publish(command);
  }

  void timerCallback(const ros::TimerEvent& event)
  {
    const ros::Time now = ros::Time::now();
    const bool fresh = trajectory_valid_ && state_valid_ && steering_valid_ &&
                       (now - trajectory_received_).toSec() <= trajectory_timeout_s_ &&
                       (now - state_received_).toSec() <= state_timeout_s_ &&
                       (now - steering_received_).toSec() <= steering_timeout_s_;
    if (!fresh)
    {
      publishStop();
      return;
    }
    if (normalizedFrame(trajectory_.header.frame_id) != normalizedFrame(state_.header.frame_id))
    {
      ROS_ERROR_THROTTLE(1.0, "longi_pi frame mismatch: trajectory=%s state=%s",
                         trajectory_.header.frame_id.c_str(), state_.header.frame_id.c_str());
      publishStop();
      return;
    }

    const std::size_t nearest = nearestIndex();
    if (nearest + 1 >= trajectory_.points.size())
    {
      publishStop();
      return;
    }
    const double target_velocity = trajectory_.points[nearest].longitudinal_velocity_mps;
    const auto& linear = state_.twist.twist.linear;
    const double current_velocity =
        std::sqrt(linear.x * linear.x + linear.y * linear.y + linear.z * linear.z) *
        kinematic_velocity_scale_to_mps_;
    const double error = target_velocity - current_velocity;
    double dt = (event.current_real - event.last_real).toSec();
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 1.0)
    {
      dt = 1.0 / control_rate_hz_;
    }

    const double candidate_integral =
        std::max(-integral_limit_, std::min(integral_limit_, integral_ + error * dt));
    const double candidate_command = kp_ * error + ki_ * candidate_integral;
    const bool drives_upper_saturation = candidate_command > max_accel_cmd_ && error > 0.0;
    const bool drives_lower_saturation = candidate_command < -max_brake_cmd_ && error < 0.0;
    if (!drives_upper_saturation && !drives_lower_saturation)
    {
      integral_ = candidate_integral;
    }
    const double command_value = kp_ * error + ki_ * integral_;

    morai_msgs::CtrlCmd command;
    command.longlCmdType = 1;
    command.steering = steering_;
    command.velocity = 0.0;
    command.acceleration = 0.0;
    if (command_value > command_deadband_)
    {
      command.accel = std::max(0.0, std::min(max_accel_cmd_, command_value));
      command.brake = 0.0;
    }
    else if (command_value < -command_deadband_)
    {
      command.accel = 0.0;
      command.brake = std::max(0.0, std::min(max_brake_cmd_, -command_value));
    }
    else
    {
      command.accel = 0.0;
      command.brake = 0.0;
    }
    command_publisher_.publish(command);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber trajectory_subscriber_;
  ros::Subscriber state_subscriber_;
  ros::Subscriber steering_subscriber_;
  ros::Publisher command_publisher_;
  ros::Timer timer_;
  std::string trajectory_topic_;
  std::string state_topic_;
  std::string steering_topic_;
  std::string ctrl_cmd_topic_;
  double control_rate_hz_{ 30.0 };
  double kinematic_velocity_scale_to_mps_{ 1.0 };
  double max_accel_cmd_{ 1.0 };
  double max_brake_cmd_{ 1.0 };
  double max_steer_rad_{ 0.698 };
  int longl_cmd_type_{ 1 };
  double trajectory_timeout_s_{ 0.5 };
  double state_timeout_s_{ 0.5 };
  double steering_timeout_s_{ 0.5 };
  int min_points_{ 2 };
  double stop_brake_cmd_{ 1.0 };
  double kp_{ 0.6 };
  double ki_{ 0.05 };
  double integral_limit_{ 3.0 };
  double command_deadband_{ 0.05 };
  double integral_{ 0.0 };
  double steering_{ 0.0 };
  bool trajectory_valid_{ false };
  bool state_valid_{ false };
  bool steering_valid_{ false };
  ros::Time trajectory_received_;
  ros::Time state_received_;
  ros::Time steering_received_;
  autoware_planning_msgs::Trajectory trajectory_;
  nav_msgs::Odometry state_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "longi_pi");
  try
  {
    LongitudinalPi controller;
    ros::spin();
  }
  catch (const std::exception& error)
  {
    ROS_FATAL("longi_pi: %s", error.what());
    return 1;
  }
  return 0;
}
