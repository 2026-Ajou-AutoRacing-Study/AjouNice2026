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
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

namespace
{
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
    private_nh_.param<std::string>("topics/ctrl_cmd", ctrl_cmd_topic_, "/ctrl_cmd");
    private_nh_.param("control_rate_hz", control_rate_hz_, 30.0);
    private_nh_.param("kinematic_velocity_scale_to_mps", velocity_scale_, 1.0);
    if (!private_nh_.getParam("vehicle/max_steer_rad", max_steer_rad_)) {
      double max_wheel_angle_deg = 40.0;
      private_nh_.param("vehicle/max_wheel_angle_deg", max_wheel_angle_deg, 40.0);
      max_steer_rad_ = max_wheel_angle_deg * M_PI / 180.0;
    }
    private_nh_.param("control_limits/max_accel_cmd", max_accel_cmd_, 1.0);
    private_nh_.param("control_limits/max_brake_cmd", max_brake_cmd_, 1.0);
    private_nh_.param("safety/trajectory_timeout_s", trajectory_timeout_s_, 0.5);
    private_nh_.param("safety/state_timeout_s", state_timeout_s_, 0.5);
    private_nh_.param("safety/steering_timeout_s", steering_timeout_s_, 0.5);
    private_nh_.param("safety/lateral_valid_timeout_s", lateral_valid_timeout_s_, 0.2);
    private_nh_.param("safety/min_points", min_points_, 3);
    private_nh_.param("safety/stop_brake_cmd", stop_brake_cmd_, 1.0);
    private_nh_.param("kp", kp_, 0.6);
    private_nh_.param("ki", ki_, 0.05);
    private_nh_.param("integral_limit", integral_limit_, 3.0);
    private_nh_.param("command_deadband", command_deadband_, 0.05);
    private_nh_.param("speed_preview/enabled", speed_preview_enabled_, true);
    private_nh_.param("speed_preview/time_s", speed_preview_time_s_, 1.5);
    private_nh_.param("speed_preview/min_distance_m", speed_preview_min_distance_m_, 5.0);
    private_nh_.param("speed_preview/max_distance_m", speed_preview_max_distance_m_, 30.0);
    private_nh_.param("stop/velocity_threshold_mps", stop_velocity_threshold_mps_, 0.1);
    private_nh_.param("stop/hold_brake_cmd", hold_brake_cmd_, 0.2);
    validateParameters();

    command_publisher_ = nh_.advertise<morai_msgs::CtrlCmd>(ctrl_cmd_topic_, 10);
    trajectory_subscriber_ = nh_.subscribe(trajectory_topic_, 2,
      &AutowareLongitudinalPi::trajectoryCallback, this);
    state_subscriber_ = nh_.subscribe(state_topic_, 10, &AutowareLongitudinalPi::stateCallback, this);
    steering_subscriber_ = nh_.subscribe(steering_topic_, 10, &AutowareLongitudinalPi::steeringCallback, this);
    validity_subscriber_ = nh_.subscribe(valid_topic_, 10, &AutowareLongitudinalPi::validityCallback, this);
    timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_hz_),
      &AutowareLongitudinalPi::timerCallback, this);
  }

private:
  void validateParameters() const
  {
    const bool valid = std::isfinite(control_rate_hz_) && control_rate_hz_ > 0.0 &&
      std::isfinite(velocity_scale_) && velocity_scale_ > 0.0 && std::isfinite(max_steer_rad_) &&
      max_steer_rad_ > 0.0 && std::isfinite(max_accel_cmd_) && max_accel_cmd_ > 0.0 &&
      max_accel_cmd_ <= 1.0 && std::isfinite(max_brake_cmd_) && max_brake_cmd_ > 0.0 &&
      max_brake_cmd_ <= 1.0 && std::isfinite(stop_brake_cmd_) && stop_brake_cmd_ >= 0.0 &&
      stop_brake_cmd_ <= 1.0 && std::isfinite(trajectory_timeout_s_) && trajectory_timeout_s_ > 0.0 &&
      std::isfinite(state_timeout_s_) && state_timeout_s_ > 0.0 && std::isfinite(steering_timeout_s_) &&
      steering_timeout_s_ > 0.0 && std::isfinite(lateral_valid_timeout_s_) && lateral_valid_timeout_s_ > 0.0 &&
      min_points_ >= 3 && std::isfinite(kp_) && kp_ >= 0.0 && std::isfinite(ki_) && ki_ >= 0.0 &&
      std::isfinite(integral_limit_) && integral_limit_ > 0.0 && std::isfinite(command_deadband_) &&
      command_deadband_ >= 0.0 && std::isfinite(speed_preview_time_s_) && speed_preview_time_s_ >= 0.0 &&
      std::isfinite(speed_preview_min_distance_m_) && speed_preview_min_distance_m_ >= 0.0 &&
      std::isfinite(speed_preview_max_distance_m_) &&
      speed_preview_max_distance_m_ >= speed_preview_min_distance_m_ &&
      std::isfinite(stop_velocity_threshold_mps_) && stop_velocity_threshold_mps_ >= 0.0 &&
      std::isfinite(hold_brake_cmd_) && hold_brake_cmd_ >= 0.0 && hold_brake_cmd_ <= max_brake_cmd_;
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
        std::isfinite(point.longitudinal_velocity_mps);
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

  double previewTargetVelocity(const std::size_t nearest, const double current_velocity) const
  {
    const double initial_target = trajectory_.points[nearest].longitudinal_velocity_mps;
    if (!speed_preview_enabled_ || initial_target < 0.0 || current_velocity < 0.0) {
      return initial_target;
    }

    const double preview_distance = std::max(
      speed_preview_min_distance_m_, std::min(
        speed_preview_max_distance_m_, std::abs(current_velocity) * speed_preview_time_s_));
    double target_velocity = initial_target;
    double traversed_distance = 0.0;
    for (std::size_t i = nearest; i + 1U < trajectory_.points.size(); ++i) {
      const auto & first = trajectory_.points[i];
      const auto & second = trajectory_.points[i + 1U];
      traversed_distance += std::hypot(
        second.pose.position.x - first.pose.position.x,
        second.pose.position.y - first.pose.position.y);
      if (traversed_distance > preview_distance) {
        break;
      }
      // A trajectory owns the speed plan.  The controller only starts following the lowest
      // forward speed that is already scheduled inside its bounded braking preview.
      if (second.longitudinal_velocity_mps >= 0.0) {
        target_velocity = std::min(target_velocity, static_cast<double>(second.longitudinal_velocity_mps));
      }
    }
    return target_velocity;
  }

  void publishHold(const double steering)
  {
    integral_ = 0.0;
    morai_msgs::CtrlCmd command;
    command.longlCmdType = 1;
    command.accel = 0.0;
    command.brake = hold_brake_cmd_;
    command.steering = steering;
    command.velocity = 0.0;
    command.acceleration = 0.0;
    command_publisher_.publish(command);
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

  void timerCallback(const ros::TimerEvent & event)
  {
    const ros::Time now = ros::Time::now();
    const bool fresh = trajectory_valid_ && state_valid_ && steering_valid_ && lateral_valid_ &&
      (now - trajectory_received_).toSec() <= trajectory_timeout_s_ &&
      (now - state_received_).toSec() <= state_timeout_s_ &&
      (now - steering_received_).toSec() <= steering_timeout_s_ &&
      (now - lateral_valid_received_).toSec() <= lateral_valid_timeout_s_;
    if (!fresh || normalizedFrame(trajectory_.header.frame_id) != normalizedFrame(state_.header.frame_id)) {
      publishStop();
      return;
    }
    const std::size_t nearest = nearestIndex();
    if (nearest + 1U >= trajectory_.points.size()) {
      publishStop();
      return;
    }
    const double current_velocity = state_.twist.twist.linear.x * velocity_scale_;
    const double target_velocity = previewTargetVelocity(nearest, current_velocity);
    if (target_velocity <= stop_velocity_threshold_mps_ &&
        std::abs(current_velocity) <= stop_velocity_threshold_mps_) {
      publishHold(steering_);
      return;
    }
    const double error = target_velocity - current_velocity;
    double dt = (event.current_real - event.last_real).toSec();
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 1.0) {
      dt = 1.0 / control_rate_hz_;
    }
    const double candidate_integral = std::max(-integral_limit_, std::min(integral_limit_, integral_ + error * dt));
    const double candidate_command = kp_ * error + ki_ * candidate_integral;
    if (!((candidate_command > max_accel_cmd_ && error > 0.0) ||
          (candidate_command < -max_brake_cmd_ && error < 0.0))) {
      integral_ = candidate_integral;
    }
    const double command_value = kp_ * error + ki_ * integral_;
    morai_msgs::CtrlCmd command;
    command.longlCmdType = 1;
    command.steering = steering_;
    command.velocity = 0.0;
    command.acceleration = 0.0;
    command.accel = command_value > command_deadband_
      ? std::min(max_accel_cmd_, command_value) : 0.0;
    command.brake = command_value < -command_deadband_
      ? std::min(max_brake_cmd_, -command_value) : 0.0;
    command_publisher_.publish(command);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber trajectory_subscriber_;
  ros::Subscriber state_subscriber_;
  ros::Subscriber steering_subscriber_;
  ros::Subscriber validity_subscriber_;
  ros::Publisher command_publisher_;
  ros::Timer timer_;
  std::string trajectory_topic_;
  std::string state_topic_;
  std::string steering_topic_;
  std::string valid_topic_;
  std::string ctrl_cmd_topic_;
  double control_rate_hz_{30.0};
  double velocity_scale_{1.0};
  double max_steer_rad_{0.698};
  double max_accel_cmd_{1.0};
  double max_brake_cmd_{1.0};
  double trajectory_timeout_s_{0.5};
  double state_timeout_s_{0.5};
  double steering_timeout_s_{0.5};
  double lateral_valid_timeout_s_{0.2};
  double stop_brake_cmd_{1.0};
  int min_points_{3};
  double kp_{0.6};
  double ki_{0.05};
  double integral_limit_{3.0};
  double command_deadband_{0.05};
  bool speed_preview_enabled_{true};
  double speed_preview_time_s_{1.5};
  double speed_preview_min_distance_m_{5.0};
  double speed_preview_max_distance_m_{30.0};
  double stop_velocity_threshold_mps_{0.1};
  double hold_brake_cmd_{0.2};
  double integral_{0.0};
  double steering_{0.0};
  bool trajectory_valid_{false};
  bool state_valid_{false};
  bool steering_valid_{false};
  bool lateral_valid_{false};
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
    ROS_FATAL("longi_pi_autoware: %s", error.what());
    return 1;
  }
  return 0;
}
