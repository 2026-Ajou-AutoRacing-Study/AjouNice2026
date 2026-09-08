#include "autoware/planning_validator/planning_validator_manager.hpp"
#include "autoware/planning_validator/invalid_trajectory_handling_core.hpp"
#include "autoware/planning_validator/debug_marker_ros1.hpp"
#include "autoware/planning_validator/diagnostic_render_core.hpp"

#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/motion_utils/vehicle/vehicle_state_checker.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <autoware_map_msgs/LaneletMapBin.h>
#include <autoware_perception_msgs/TrafficLightGroupArray.h>
#include <autoware_planning_msgs/LaneletRoute.h>
#include <autoware/route_handler/route_handler.hpp>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <memory>
#include <stdexcept>

namespace
{
class PlanningValidatorCandidateNode
{
public:
  PlanningValidatorCandidateNode()
  : private_nh_("~"), tf_buffer_(), tf_listener_(tf_buffer_)
  {
    debug_marker_.reset(new autoware::planning_validator::DebugMarkerRos1(private_nh_));
    trajectory_sub_ = private_nh_.subscribe("input/trajectory", 1, &PlanningValidatorCandidateNode::on_trajectory, this);
    kinematics_sub_ = private_nh_.subscribe("input/kinematics", 1, &PlanningValidatorCandidateNode::on_kinematics, this);
    acceleration_sub_ = private_nh_.subscribe("input/acceleration", 1, &PlanningValidatorCandidateNode::on_acceleration, this);
    pointcloud_sub_ = private_nh_.subscribe("input/pointcloud", 1, &PlanningValidatorCandidateNode::on_pointcloud, this);
    route_sub_ = private_nh_.subscribe("input/route", 1, &PlanningValidatorCandidateNode::on_route, this);
    map_sub_ = private_nh_.subscribe("input/lanelet_map_bin", 1, &PlanningValidatorCandidateNode::on_map, this);
    traffic_signals_sub_ = private_nh_.subscribe("input/traffic_signals", 1, &PlanningValidatorCandidateNode::on_traffic_signals, this);
    output_trajectory_pub_ = private_nh_.advertise<autoware_planning_msgs::Trajectory>("output/trajectory", 1, false);
    status_pub_ = private_nh_.advertise<autoware_planning_validator::PlanningValidatorStatus>("output/validation_status", 1, false);
    diagnostics_pub_ = private_nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1, false);
    vehicle_stop_checker_.reset(new autoware::motion_utils::VehicleStopCheckerBase(&private_nh_, 10.0));
    private_nh_.param("trajectory_checker_plugin", trajectory_plugin_name_, std::string{"autoware::planning_validator::TrajectoryCheckerRos1"});
    private_nh_.param("latency_checker_plugin", latency_plugin_name_, std::string{"autoware::planning_validator::LatencyCheckerRos1"});
    private_nh_.param("intersection_collision_checker_plugin", intersection_plugin_name_, std::string{"autoware::planning_validator::IntersectionCollisionCheckerRos1"});
    private_nh_.param("rear_collision_checker_plugin", rear_plugin_name_, std::string{"autoware::planning_validator::RearCollisionCheckerRos1"});
    private_nh_.param("publish_diag", publish_diag_, true);
    private_nh_.param("diag_error_count_threshold", diagnostic_parameters_.error_count_threshold, 0);
    int handling_value = static_cast<int>(
      autoware::planning_validator::InvalidTrajectoryHandlingType::PublishAsIs);
    private_nh_.param("default_handling_type", handling_value, handling_value);
    if (handling_value < 0 || handling_value > 2) {
      ROS_ERROR_STREAM("planning validator starts fail-closed: invalid default_handling_type=" << handling_value);
    } else {
      handling_parameters_.default_handling_type =
        static_cast<autoware::planning_validator::InvalidTrajectoryHandlingType>(handling_value);
      private_nh_.param(
        "soft_stop_deceleration", handling_parameters_.soft_stop_deceleration,
        handling_parameters_.soft_stop_deceleration);
      private_nh_.param(
        "soft_stop_jerk_lim", handling_parameters_.soft_stop_jerk_lim,
        handling_parameters_.soft_stop_jerk_lim);
      handling_parameters_ready_ = true;
    }
    try {
      vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(private_nh_).getVehicleInfo();
      vehicle_info_ready_ = true;
    } catch (const std::exception & exception) {
      ROS_ERROR_STREAM("planning validator R002 starts fail-closed: vehicle-info parameters unavailable: " << exception.what());
    }
    plugin_loaded_ = manager_.load_plugin(private_nh_, trajectory_plugin_name_) &&
      manager_.load_plugin(private_nh_, latency_plugin_name_) &&
      manager_.load_plugin(private_nh_, intersection_plugin_name_) &&
      manager_.load_plugin(private_nh_, rear_plugin_name_);
    if (!plugin_loaded_) {
      ROS_ERROR("planning validator starts fail-closed: cannot load configured checker plugin");
    } else {
      ROS_INFO_STREAM("planning validator candidate loaded " << manager_.loaded_plugin_count()
        << " configured providers");
    }
  }

private:
  void on_kinematics(const nav_msgs::Odometry::ConstPtr & message)
  {
    current_kinematics_ = message;
    geometry_msgs::TwistStamped velocity;
    velocity.header = message->header;
    velocity.twist = message->twist.twist;
    vehicle_stop_checker_->addTwist(velocity);
    kinematics_ready_ = true;
  }
  void on_acceleration(const geometry_msgs::AccelWithCovarianceStamped::ConstPtr & message)
  {
    acceleration_message_ = message;
    acceleration_ready_ = true;
  }
  void on_pointcloud(const sensor_msgs::PointCloud2::ConstPtr & message)
  {
    pointcloud_message_ = message;
    pointcloud_ready_ = true;
  }
  void on_traffic_signals(const autoware_perception_msgs::TrafficLightGroupArray::ConstPtr & message)
  {
    traffic_signals_message_ = message;
  }
  void on_route(const autoware_planning_msgs::LaneletRoute::ConstPtr & message)
  {
    route_message_ = message;
    update_route_context();
  }
  void on_map(const autoware_map_msgs::LaneletMapBin::ConstPtr & message)
  {
    map_message_ = message;
    update_route_context();
  }
  void update_route_context()
  {
    route_ready_ = false;
    map_ready_ = false;
    if (!route_message_ || !map_message_) return;
    try {
      route_handler_.setMap(*map_message_);
      map_ready_ = route_handler_.isMapMsgReady();
      route_handler_.setRoute(*route_message_);
      route_ready_ = route_handler_.isHandlerReady();
    } catch (const std::exception & exception) {
      ROS_ERROR_STREAM("planning validator R002 route/map update failed: " << exception.what());
    }
  }
  bool all_mandatory_inputs_ready() const
  {
    return plugin_loaded_ && manager_.loaded_plugin_count() == 4U && handling_parameters_ready_ && vehicle_info_ready_ && kinematics_ready_ &&
      acceleration_ready_ && pointcloud_ready_ && route_ready_ && map_ready_;
  }
  static bool all_valid(const autoware_planning_validator::PlanningValidatorStatus & status)
  {
    return status.is_valid_size && status.is_valid_finite_value && status.is_valid_interval &&
      status.is_valid_relative_angle && status.is_valid_curvature && status.is_valid_lateral_acc &&
      status.is_valid_lateral_jerk && status.is_valid_longitudinal_max_acc &&
      status.is_valid_longitudinal_min_acc && status.is_valid_steering && status.is_valid_steering_rate &&
      status.is_valid_velocity_deviation && status.is_valid_distance_deviation &&
      status.is_valid_longitudinal_distance_deviation && status.is_valid_forward_trajectory_length &&
      status.is_valid_latency && status.is_valid_yaw_deviation && status.is_valid_trajectory_shift &&
      status.is_valid_intersection_collision_check && status.is_valid_rear_collision_check;
  }
  static bool has_only_finite_trajectory_values(const autoware_planning_msgs::Trajectory & trajectory)
  {
    for (const auto & point : trajectory.points) {
      const auto & position = point.pose.position;
      const auto & orientation = point.pose.orientation;
      if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) ||
        !std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
        !std::isfinite(orientation.z) || !std::isfinite(orientation.w) ||
        !std::isfinite(point.longitudinal_velocity_mps) || !std::isfinite(point.heading_rate_rps) ||
        !std::isfinite(point.acceleration_mps2)) return false;
    }
    return true;
  }
  static autoware_planning_validator::PlanningValidatorStatus finite_input_failure(
    const autoware_planning_msgs::Trajectory & trajectory)
  {
    // Source initializes all checks before providers run. Preserve that
    // lifecycle for a non-finite input instead of making resampling invent a
    // trajectory-shift failure.
    autoware_planning_validator::PlanningValidatorStatus status;
    status.stamp = trajectory.header.stamp;
    status.trajectory_size = static_cast<int64_t>(trajectory.points.size());
    status.is_valid_size = trajectory.points.size() >= 2U;
    status.is_valid_finite_value = false;
    status.is_valid_interval = status.is_valid_relative_angle = status.is_valid_curvature = true;
    status.is_valid_lateral_acc = status.is_valid_lateral_jerk = true;
    status.is_valid_longitudinal_max_acc = status.is_valid_longitudinal_min_acc = true;
    status.is_valid_steering = status.is_valid_steering_rate = true;
    status.is_valid_velocity_deviation = status.is_valid_distance_deviation = true;
    status.is_valid_longitudinal_distance_deviation = status.is_valid_forward_trajectory_length = true;
    status.is_valid_latency = status.is_valid_yaw_deviation = status.is_valid_trajectory_shift = true;
    status.is_valid_intersection_collision_check = status.is_valid_rear_collision_check = true;
    return status;
  }
  void on_trajectory(const autoware_planning_msgs::Trajectory::ConstPtr & trajectory)
  {
    if (!has_only_finite_trajectory_values(*trajectory)) {
      if (!all_mandatory_inputs_ready()) {
        ROS_WARN_THROTTLE(5.0, "planning validator R002 waiting for mandatory route/map/kinematics/acceleration/pointcloud inputs");
        return;
      }
      auto status = finite_input_failure(*trajectory);
      ++invalid_count_;
      status.invalid_count = invalid_count_;
      status_pub_.publish(status);
      if (publish_diag_) {
        diagnostic_parameters_.trajectory_override_all_errors = false;
        diagnostic_msgs::DiagnosticArray diagnostics;
        diagnostics.header.stamp = status.stamp;
        diagnostics.status = autoware::planning_validator::render_source_shaped_diagnostics(
          status, diagnostic_parameters_);
        diagnostics_pub_.publish(diagnostics);
      }
      debug_marker_->publish_invalid(current_kinematics_->pose.pose, status);
      const auto selected = autoware::planning_validator::select_invalid_trajectory(
        *trajectory, last_valid_trajectory_.get(), handling_parameters_.default_handling_type);
      if (selected) output_trajectory_pub_.publish(*selected);
      else ROS_ERROR_THROTTLE(3.0, "planning validator invalid trajectory: selected source policy requires a previous valid trajectory");
      return;
    }
    try {
      // This is the same 1 m source-context resampling interval used by the
      // ROS2 validator before its collision checker consumes a trajectory.
      resampled_trajectory_.reset(
        new autoware_planning_msgs::Trajectory(
          autoware::motion_utils::resampleTrajectory(*trajectory, 1.0)));
    } catch (const std::exception & exception) {
      resampled_trajectory_.reset();
      ROS_ERROR_STREAM("planning validator R002 cannot resample trajectory: " << exception.what());
      return;
    }
    if (!all_mandatory_inputs_ready()) {
      ROS_WARN_THROTTLE(5.0, "planning validator R002 waiting for mandatory route/map/kinematics/acceleration/pointcloud inputs");
      return;
    }
    auto status = manager_.validate({
      *trajectory, current_kinematics_.get(), pointcloud_message_.get(), acceleration_message_.get(),
      route_message_.get(), map_message_.get(), traffic_signals_message_.get(),
      resampled_trajectory_.get(), &vehicle_info_, &tf_buffer_, &route_handler_, last_valid_trajectory_.get(),
      vehicle_stop_checker_->isVehicleStopped()});
    if (all_valid(status)) {
      invalid_count_ = 0;
    } else {
      ++invalid_count_;
    }
    status.invalid_count = invalid_count_;
    status_pub_.publish(status);
    if (publish_diag_) {
      diagnostic_parameters_.trajectory_override_all_errors = manager_.diagnostic_override_all_errors();
      diagnostic_msgs::DiagnosticArray diagnostics;
      diagnostics.header.stamp = status.stamp;
      diagnostics.status = autoware::planning_validator::render_source_shaped_diagnostics(
        status, diagnostic_parameters_);
      diagnostics_pub_.publish(diagnostics);
    }
    if (!all_valid(status)) debug_marker_->publish_invalid(current_kinematics_->pose.pose, status);
    if (all_valid(status)) {
      output_trajectory_pub_.publish(*trajectory);
      last_valid_trajectory_.reset(new autoware_planning_msgs::Trajectory(*trajectory));
      soft_stop_trajectory_.reset();
      return;
    }

    const int provider_handling = manager_.requested_invalid_trajectory_handling();
    if (provider_handling == -2) {
      ROS_ERROR_THROTTLE(3.0, "planning validator invalid trajectory: provider handling policy is malformed");
      return;
    }
    auto type = handling_parameters_.default_handling_type;
    if (provider_handling >= 0 && provider_handling > static_cast<int>(type)) {
      type = static_cast<autoware::planning_validator::InvalidTrajectoryHandlingType>(provider_handling);
    }
    if (type == autoware::planning_validator::InvalidTrajectoryHandlingType::UsePreviousResultWithSoftStop &&
      last_valid_trajectory_ && !soft_stop_trajectory_) {
      const auto nearest_index = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
        last_valid_trajectory_->points, current_kinematics_->pose.pose);
      soft_stop_trajectory_.reset(new autoware_planning_msgs::Trajectory(
        autoware::planning_validator::make_soft_stop_trajectory(
          *last_valid_trajectory_, nearest_index, current_kinematics_->twist.twist.linear.x,
          acceleration_message_->accel.accel.linear.x, handling_parameters_)));
    }
    const auto * selected_previous =
      type == autoware::planning_validator::InvalidTrajectoryHandlingType::UsePreviousResultWithSoftStop &&
      soft_stop_trajectory_ ? soft_stop_trajectory_.get() : last_valid_trajectory_.get();
    const auto selected = autoware::planning_validator::select_invalid_trajectory(
      *trajectory, selected_previous, type);
    if (selected) {
      output_trajectory_pub_.publish(*selected);
    } else {
      ROS_ERROR_THROTTLE(3.0, "planning validator invalid trajectory: selected source policy requires a previous valid trajectory");
    }
  }

  ros::NodeHandle private_nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  autoware::planning_validator::PlanningValidatorManager manager_;
  std::string trajectory_plugin_name_, latency_plugin_name_, intersection_plugin_name_, rear_plugin_name_;
  bool plugin_loaded_{false};
  bool publish_diag_{true};
  bool handling_parameters_ready_{false};
  bool vehicle_info_ready_{false};
  bool kinematics_ready_{false}, acceleration_ready_{false}, pointcloud_ready_{false};
  int64_t invalid_count_{0};
  bool route_ready_{false}, map_ready_{false};
  nav_msgs::Odometry::ConstPtr current_kinematics_;
  geometry_msgs::AccelWithCovarianceStamped::ConstPtr acceleration_message_;
  sensor_msgs::PointCloud2::ConstPtr pointcloud_message_;
  autoware_planning_msgs::LaneletRoute::ConstPtr route_message_;
  autoware_map_msgs::LaneletMapBin::ConstPtr map_message_;
  autoware_perception_msgs::TrafficLightGroupArray::ConstPtr traffic_signals_message_;
  autoware_planning_msgs::Trajectory::Ptr resampled_trajectory_;
  autoware_planning_msgs::Trajectory::Ptr last_valid_trajectory_;
  autoware_planning_msgs::Trajectory::Ptr soft_stop_trajectory_;
  autoware::planning_validator::InvalidTrajectoryHandlingParameters handling_parameters_;
  autoware::planning_validator::DiagnosticRenderParameters diagnostic_parameters_;
  std::unique_ptr<autoware::planning_validator::DebugMarkerRos1> debug_marker_;
  std::unique_ptr<autoware::motion_utils::VehicleStopCheckerBase> vehicle_stop_checker_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
  autoware::route_handler::RouteHandler route_handler_;
  ros::Subscriber trajectory_sub_, kinematics_sub_, acceleration_sub_, pointcloud_sub_, route_sub_, map_sub_, traffic_signals_sub_;
  ros::Publisher output_trajectory_pub_, status_pub_, diagnostics_pub_;
};
}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "planning_validator_candidate");
  PlanningValidatorCandidateNode node;
  ros::spin();
  return 0;
}
