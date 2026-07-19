#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>

#include <autoware_planning_msgs/Trajectory.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace
{

bool finitePose(const geometry_msgs::Pose& pose)
{
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) && std::isfinite(pose.position.z) &&
         std::isfinite(pose.orientation.x) && std::isfinite(pose.orientation.y) &&
         std::isfinite(pose.orientation.z) && std::isfinite(pose.orientation.w);
}

double distance2d(const geometry_msgs::Point& first, const geometry_msgs::Point& second)
{
  return std::hypot(second.x - first.x, second.y - first.y);
}

std::string normalizedFrame(const std::string& frame)
{
  const std::size_t first = frame.find_first_not_of('/');
  return first == std::string::npos ? std::string() : frame.substr(first);
}

bool finiteState(const nav_msgs::Odometry& state)
{
  const auto& pose = state.pose.pose;
  const auto& linear = state.twist.twist.linear;
  const double quaternion_norm = pose.orientation.x * pose.orientation.x +
                                 pose.orientation.y * pose.orientation.y +
                                 pose.orientation.z * pose.orientation.z +
                                 pose.orientation.w * pose.orientation.w;
  return !normalizedFrame(state.header.frame_id).empty() && finitePose(pose) &&
         std::isfinite(quaternion_norm) && quaternion_norm > 1e-12 &&
         std::isfinite(linear.x) && std::isfinite(linear.y) && std::isfinite(linear.z);
}

double yawFromQuaternion(const geometry_msgs::Quaternion& quaternion)
{
  const double sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cos_yaw = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(sin_yaw, cos_yaw);
}

}  // namespace

class PurePursuit
{
public:
  PurePursuit() : private_nh_("~")
  {
    private_nh_.param<std::string>("topics/trajectory", trajectory_topic_, "/control/trajectory");
    private_nh_.param<std::string>("topics/kinematic_state", state_topic_, "/localization/kinematic_state");
    private_nh_.param<std::string>("topics/steering_cmd", steering_topic_, "/control/command/steering");
    private_nh_.param("control_rate_hz", control_rate_hz_, 30.0);
    private_nh_.param("kinematic_velocity_scale_to_mps", kinematic_velocity_scale_to_mps_, 1.0);
    private_nh_.param("vehicle/wheelbase_m", wheelbase_m_, 3.0);
    private_nh_.param("vehicle/max_steer_rad", max_steer_rad_, 0.698);
    private_nh_.param("safety/trajectory_timeout_s", trajectory_timeout_s_, 0.5);
    private_nh_.param("safety/state_timeout_s", state_timeout_s_, 0.5);
    private_nh_.param("safety/min_points", min_points_, 2);
    private_nh_.param("min_lookahead_m", min_lookahead_m_, 4.0);
    private_nh_.param("lookahead_time_s", lookahead_time_s_, 1.5);
    private_nh_.param("max_lookahead_m", max_lookahead_m_, 20.0);
    private_nh_.param("steering_gain", steering_gain_, 1.0);
    validateParameters();

    steering_publisher_ = nh_.advertise<std_msgs::Float64>(steering_topic_, 10);
    trajectory_subscriber_ =
        nh_.subscribe(trajectory_topic_, 2, &PurePursuit::trajectoryCallback, this);
    state_subscriber_ = nh_.subscribe(state_topic_, 10, &PurePursuit::stateCallback, this);
    timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_hz_), &PurePursuit::timerCallback, this);
    ROS_INFO_STREAM("purepursuit: trajectory=" << trajectory_topic_ << " state=" << state_topic_ << " -> "
                                               << steering_topic_ << " at " << control_rate_hz_ << " Hz");
  }

private:
  void validateParameters() const
  {
    const bool valid = std::isfinite(control_rate_hz_) && control_rate_hz_ > 0.0 &&
                       std::isfinite(kinematic_velocity_scale_to_mps_) &&
                       kinematic_velocity_scale_to_mps_ > 0.0 && std::isfinite(wheelbase_m_) &&
                       wheelbase_m_ > 0.0 && std::isfinite(max_steer_rad_) && max_steer_rad_ > 0.0 &&
                       std::isfinite(min_lookahead_m_) && min_lookahead_m_ > 0.0 &&
                       std::isfinite(max_lookahead_m_) && max_lookahead_m_ >= min_lookahead_m_ &&
                       std::isfinite(lookahead_time_s_) && lookahead_time_s_ >= 0.0 &&
                       std::isfinite(steering_gain_) && steering_gain_ > 0.0 &&
                       std::isfinite(trajectory_timeout_s_) && trajectory_timeout_s_ > 0.0 &&
                       std::isfinite(state_timeout_s_) && state_timeout_s_ > 0.0 && min_points_ >= 2;
    if (!valid)
    {
      throw std::runtime_error("invalid Pure Pursuit parameter");
    }
  }

  void trajectoryCallback(const autoware_planning_msgs::Trajectory::ConstPtr& message)
  {
    bool valid = !normalizedFrame(message->header.frame_id).empty() &&
                 message->points.size() >= static_cast<std::size_t>(min_points_);
    for (const auto& point : message->points)
    {
      valid = valid && finitePose(point.pose) && std::isfinite(point.longitudinal_velocity_mps) &&
              point.longitudinal_velocity_mps >= 0.0F;
    }
    if (!valid)
    {
      trajectory_valid_ = false;
      ROS_WARN_THROTTLE(1.0, "purepursuit rejected invalid trajectory");
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
      ROS_WARN_THROTTLE(1.0, "purepursuit rejected invalid kinematic state");
      return;
    }
    state_ = *message;
    state_valid_ = true;
    state_received_ = ros::Time::now();
  }

  std::size_t nearestIndex() const
  {
    std::size_t nearest = 0;
    double best_squared_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < trajectory_.points.size(); ++index)
    {
      const auto& position = trajectory_.points[index].pose.position;
      const double dx = position.x - state_.pose.pose.position.x;
      const double dy = position.y - state_.pose.pose.position.y;
      const double squared_distance = dx * dx + dy * dy;
      if (squared_distance < best_squared_distance)
      {
        best_squared_distance = squared_distance;
        nearest = index;
      }
    }
    return nearest;
  }

  std::size_t targetIndex(const std::size_t nearest, const double lookahead) const
  {
    double accumulated = 0.0;
    for (std::size_t index = nearest + 1; index < trajectory_.points.size(); ++index)
    {
      accumulated += distance2d(trajectory_.points[index - 1].pose.position,
                                trajectory_.points[index].pose.position);
      if (accumulated >= lookahead)
      {
        return index;
      }
    }
    return trajectory_.points.size() - 1;
  }

  void publishZero()
  {
    std_msgs::Float64 output;
    output.data = 0.0;
    steering_publisher_.publish(output);
  }

  void timerCallback(const ros::TimerEvent&)
  {
    const ros::Time now = ros::Time::now();
    const bool fresh = trajectory_valid_ && state_valid_ &&
                       (now - trajectory_received_).toSec() <= trajectory_timeout_s_ &&
                       (now - state_received_).toSec() <= state_timeout_s_;
    if (!fresh)
    {
      publishZero();
      return;
    }
    if (normalizedFrame(trajectory_.header.frame_id) != normalizedFrame(state_.header.frame_id))
    {
      ROS_ERROR_THROTTLE(1.0, "purepursuit frame mismatch: trajectory=%s state=%s",
                         trajectory_.header.frame_id.c_str(), state_.header.frame_id.c_str());
      publishZero();
      return;
    }
    const auto& linear = state_.twist.twist.linear;
    const double speed =
        std::sqrt(linear.x * linear.x + linear.y * linear.y + linear.z * linear.z) *
        kinematic_velocity_scale_to_mps_;
    const double lookahead =
        std::max(min_lookahead_m_, std::min(max_lookahead_m_, min_lookahead_m_ + speed * lookahead_time_s_));
    const std::size_t nearest = nearestIndex();
    const std::size_t target = targetIndex(nearest, lookahead);
    const auto& target_position = trajectory_.points[target].pose.position;
    const double yaw = yawFromQuaternion(state_.pose.pose.orientation);
    const double dx = target_position.x - state_.pose.pose.position.x;
    const double dy = target_position.y - state_.pose.pose.position.y;
    const double target_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double target_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    if (!std::isfinite(target_x) || !std::isfinite(target_y) || target_x <= 0.0)
    {
      publishZero();
      return;
    }
    const double curvature = 2.0 * target_y / (lookahead * lookahead);
    const double raw_steering = std::atan(wheelbase_m_ * curvature) * steering_gain_;
    std_msgs::Float64 output;
    output.data = std::max(-max_steer_rad_, std::min(max_steer_rad_, raw_steering));
    steering_publisher_.publish(output);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber trajectory_subscriber_;
  ros::Subscriber state_subscriber_;
  ros::Publisher steering_publisher_;
  ros::Timer timer_;
  std::string trajectory_topic_;
  std::string state_topic_;
  std::string steering_topic_;
  double control_rate_hz_{ 30.0 };
  double kinematic_velocity_scale_to_mps_{ 1.0 };
  double wheelbase_m_{ 3.0 };
  double max_steer_rad_{ 0.698 };
  double trajectory_timeout_s_{ 0.5 };
  double state_timeout_s_{ 0.5 };
  int min_points_{ 2 };
  double min_lookahead_m_{ 4.0 };
  double lookahead_time_s_{ 1.5 };
  double max_lookahead_m_{ 20.0 };
  double steering_gain_{ 1.0 };
  bool trajectory_valid_{ false };
  bool state_valid_{ false };
  ros::Time trajectory_received_;
  ros::Time state_received_;
  autoware_planning_msgs::Trajectory trajectory_;
  nav_msgs::Odometry state_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "purepursuit");
  try
  {
    PurePursuit controller;
    ros::spin();
  }
  catch (const std::exception& error)
  {
    ROS_FATAL("purepursuit: %s", error.what());
    return 1;
  }
  return 0;
}
