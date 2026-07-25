#include "trajectory_follower/adaptive_pure_pursuit_core.hpp"

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <autoware_planning_msgs/Trajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

namespace
{
std::string normalizedFrame(const std::string & frame)
{
  const std::size_t first = frame.find_first_not_of('/');
  return first == std::string::npos ? std::string() : frame.substr(first);
}

bool finitePose(const geometry_msgs::Pose & pose)
{
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.orientation.x) && std::isfinite(pose.orientation.y) &&
         std::isfinite(pose.orientation.z) && std::isfinite(pose.orientation.w);
}

double yawFromQuaternion(const geometry_msgs::Quaternion & quaternion)
{
  const double sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cos_yaw = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(sin_yaw, cos_yaw);
}
}  // namespace

class AutowarePurePursuitNode
{
public:
  AutowarePurePursuitNode() : private_nh_("~")
  {
    private_nh_.param<std::string>("topics/trajectory", trajectory_topic_, "/control/trajectory");
    private_nh_.param<std::string>("topics/kinematic_state", state_topic_, "/localization/kinematic_state");
    private_nh_.param<std::string>("topics/steering_cmd", steering_topic_, "/control/command/steering");
    private_nh_.param<std::string>("topics/controller_valid", valid_topic_,
                                   "/control/autoware/controller_valid");
    private_nh_.param<std::string>("topics/failure_reason", failure_topic_,
                                   "/control/autoware/failure_reason");
    private_nh_.param<std::string>("topics/target_point", target_point_topic_,
                                   "/control/autoware/target_point");
    private_nh_.param("control_rate_hz", control_rate_hz_, 30.0);
    private_nh_.param("kinematic_velocity_scale_to_mps", velocity_scale_, 1.0);
    private_nh_.param("safety/trajectory_timeout_s", trajectory_timeout_s_, 0.5);
    private_nh_.param("safety/state_timeout_s", state_timeout_s_, 0.5);
    private_nh_.param("safety/min_points", min_points_, 3);

    private_nh_.param("vehicle/wheelbase_m", params_.wheel_base, 3.0);
    if (!private_nh_.getParam("vehicle/max_steer_rad", params_.max_steering_angle)) {
      double max_wheel_angle_deg = 40.0;
      private_nh_.param("vehicle/max_wheel_angle_deg", max_wheel_angle_deg, 40.0);
      params_.max_steering_angle = max_wheel_angle_deg * M_PI / 180.0;
    }
    private_nh_.param("adaptive/ld_velocity_ratio", params_.ld_velocity_ratio, 2.4);
    private_nh_.param("adaptive/ld_lateral_error_ratio", params_.ld_lateral_error_ratio, 3.6);
    private_nh_.param("adaptive/ld_curvature_ratio", params_.ld_curvature_ratio, 120.0);
    private_nh_.param("adaptive/long_ld_lateral_error_threshold",
                      params_.long_ld_lateral_error_threshold, 0.5);
    private_nh_.param("adaptive/min_lookahead_distance", params_.min_lookahead_distance, 4.35);
    private_nh_.param("adaptive/max_lookahead_distance", params_.max_lookahead_distance, 15.0);
    private_nh_.param("adaptive/reverse_min_lookahead_distance",
                      params_.reverse_min_lookahead_distance, 7.0);
    private_nh_.param("adaptive/resampling_ds", params_.resampling_ds, 0.1);
    private_nh_.param("adaptive/curvature_calculation_distance",
                      params_.curvature_calculation_distance, 4.0);

    core_.reset(new trajectory_follower::AdaptivePurePursuitCore(params_));
    validateParameters();

    steering_publisher_ = nh_.advertise<std_msgs::Float64>(steering_topic_, 10);
    valid_publisher_ = nh_.advertise<std_msgs::Bool>(valid_topic_, 10);
    failure_publisher_ = nh_.advertise<std_msgs::String>(failure_topic_, 10);
    target_point_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(target_point_topic_, 10);
    lookahead_publisher_ = nh_.advertise<std_msgs::Float64>("~/debug/lookahead_distance", 10);
    lateral_error_publisher_ = nh_.advertise<std_msgs::Float64>("~/debug/lateral_error", 10);
    path_curvature_publisher_ = nh_.advertise<std_msgs::Float64>("~/debug/path_curvature", 10);
    target_curvature_publisher_ = nh_.advertise<std_msgs::Float64>("~/debug/target_curvature", 10);
    raw_steering_publisher_ = nh_.advertise<std_msgs::Float64>("~/debug/limited_steering_angle", 10);

    trajectory_subscriber_ = nh_.subscribe(trajectory_topic_, 2,
      &AutowarePurePursuitNode::trajectoryCallback, this);
    state_subscriber_ = nh_.subscribe(state_topic_, 10, &AutowarePurePursuitNode::stateCallback, this);
    timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_hz_),
      &AutowarePurePursuitNode::timerCallback, this);
    ROS_INFO_STREAM("purepursuit_autoware: " << trajectory_topic_ << " + " << state_topic_ << " -> "
                    << steering_topic_);
  }

private:
  void validateParameters() const
  {
    std::string reason;
    const bool valid = std::isfinite(control_rate_hz_) && control_rate_hz_ > 0.0 &&
      std::isfinite(velocity_scale_) && velocity_scale_ > 0.0 &&
      std::isfinite(trajectory_timeout_s_) && trajectory_timeout_s_ > 0.0 &&
      std::isfinite(state_timeout_s_) && state_timeout_s_ > 0.0 && min_points_ >= 3 &&
      core_->parametersValid(&reason);
    if (!valid) {
      throw std::runtime_error(reason.empty() ? "invalid purepursuit_autoware parameter" : reason);
    }
  }

  void trajectoryCallback(const autoware_planning_msgs::Trajectory::ConstPtr & message)
  {
    bool valid = !normalizedFrame(message->header.frame_id).empty() &&
      message->points.size() >= static_cast<std::size_t>(min_points_);
    for (const auto & point : message->points) {
      valid = valid && finitePose(point.pose) && std::isfinite(point.longitudinal_velocity_mps);
    }
    if (!valid) {
      trajectory_valid_ = false;
      ROS_WARN_THROTTLE(1.0, "purepursuit_autoware rejected invalid trajectory");
      return;
    }
    trajectory_ = *message;
    trajectory_valid_ = true;
    trajectory_received_ = ros::Time::now();
  }

  void stateCallback(const nav_msgs::Odometry::ConstPtr & message)
  {
    const auto & linear = message->twist.twist.linear;
    const bool valid = !normalizedFrame(message->header.frame_id).empty() && finitePose(message->pose.pose) &&
      std::isfinite(linear.x) && std::isfinite(linear.y) && std::isfinite(linear.z);
    if (!valid) {
      state_valid_ = false;
      ROS_WARN_THROTTLE(1.0, "purepursuit_autoware rejected invalid kinematic state");
      return;
    }
    state_ = *message;
    state_valid_ = true;
    state_received_ = ros::Time::now();
  }

  void publishResult(const trajectory_follower::PurePursuitOutput & output, const std::string & frame_id)
  {
    std_msgs::Float64 steering;
    steering.data = output.steering_tire_angle_rad;
    steering_publisher_.publish(steering);
    std_msgs::Bool valid;
    valid.data = output.valid;
    valid_publisher_.publish(valid);
    std_msgs::String failure;
    failure.data = output.failure_reason;
    failure_publisher_.publish(failure);

    std_msgs::Float64 value;
    value.data = output.lookahead_distance;
    lookahead_publisher_.publish(value);
    value.data = output.lateral_error;
    lateral_error_publisher_.publish(value);
    value.data = output.path_curvature;
    path_curvature_publisher_.publish(value);
    value.data = output.target_curvature;
    target_curvature_publisher_.publish(value);
    value.data = output.steering_tire_angle_rad;
    raw_steering_publisher_.publish(value);

    if (output.valid) {
      geometry_msgs::PoseStamped target;
      target.header.stamp = ros::Time::now();
      target.header.frame_id = frame_id;
      target.pose.position.x = output.target_point.x;
      target.pose.position.y = output.target_point.y;
      target.pose.orientation.w = 1.0;
      target_point_publisher_.publish(target);
    }
  }

  void timerCallback(const ros::TimerEvent &)
  {
    const ros::Time now = ros::Time::now();
    if (!trajectory_valid_ || !state_valid_ ||
        (now - trajectory_received_).toSec() > trajectory_timeout_s_ ||
        (now - state_received_).toSec() > state_timeout_s_) {
      trajectory_follower::PurePursuitOutput output;
      output.failure_reason = "stale or unavailable trajectory/state";
      publishResult(output, trajectory_.header.frame_id);
      return;
    }
    if (normalizedFrame(trajectory_.header.frame_id) != normalizedFrame(state_.header.frame_id)) {
      trajectory_follower::PurePursuitOutput output;
      output.failure_reason = "trajectory and state frame mismatch";
      publishResult(output, trajectory_.header.frame_id);
      return;
    }

    std::vector<trajectory_follower::PathPoint> path;
    path.reserve(trajectory_.points.size());
    for (const auto & point : trajectory_.points) {
      trajectory_follower::PathPoint converted;
      converted.pose.x = point.pose.position.x;
      converted.pose.y = point.pose.position.y;
      converted.pose.yaw = yawFromQuaternion(point.pose.orientation);
      converted.target_velocity_mps = point.longitudinal_velocity_mps;
      path.push_back(converted);
    }
    trajectory_follower::VehicleState vehicle;
    vehicle.pose.x = state_.pose.pose.position.x;
    vehicle.pose.y = state_.pose.pose.position.y;
    vehicle.pose.yaw = yawFromQuaternion(state_.pose.pose.orientation);
    // Universe uses the signed base-link longitudinal velocity for control LD.
    vehicle.velocity_mps = state_.twist.twist.linear.x * velocity_scale_;
    const auto output = core_->run(path, vehicle);
    publishResult(output, trajectory_.header.frame_id);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber trajectory_subscriber_;
  ros::Subscriber state_subscriber_;
  ros::Publisher steering_publisher_;
  ros::Publisher valid_publisher_;
  ros::Publisher failure_publisher_;
  ros::Publisher target_point_publisher_;
  ros::Publisher lookahead_publisher_;
  ros::Publisher lateral_error_publisher_;
  ros::Publisher path_curvature_publisher_;
  ros::Publisher target_curvature_publisher_;
  ros::Publisher raw_steering_publisher_;
  ros::Timer timer_;
  std::string trajectory_topic_;
  std::string state_topic_;
  std::string steering_topic_;
  std::string valid_topic_;
  std::string failure_topic_;
  std::string target_point_topic_;
  double control_rate_hz_{30.0};
  double velocity_scale_{1.0};
  double trajectory_timeout_s_{0.5};
  double state_timeout_s_{0.5};
  int min_points_{3};
  trajectory_follower::AdaptiveLookaheadParams params_;
  std::unique_ptr<trajectory_follower::AdaptivePurePursuitCore> core_;
  bool trajectory_valid_{false};
  bool state_valid_{false};
  ros::Time trajectory_received_;
  ros::Time state_received_;
  autoware_planning_msgs::Trajectory trajectory_;
  nav_msgs::Odometry state_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "purepursuit_autoware");
  try {
    AutowarePurePursuitNode node;
    ros::spin();
  } catch (const std::exception & error) {
    ROS_FATAL("purepursuit_autoware: %s", error.what());
    return 1;
  }
  return 0;
}
