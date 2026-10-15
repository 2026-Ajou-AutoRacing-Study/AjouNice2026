// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/planning_validator/full_source/node.hpp"

#include "autoware/planning_validator/full_source/utils.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>

#include <angles/angles.h>

#include <autoware/motion_utils/trajectory/static_obstacle_collision.hpp>

#include <memory>
#include <boost/crc.hpp>
#include <ros/serialization.h>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <string>

namespace autoware::planning_validator::full_source
{
using diagnostic_msgs::DiagnosticStatus;

PlanningValidatorNode::PlanningValidatorNode(const ros::NodeHandle & options)
: validator_native::Node(options)
{
  // trajectory subscriber
  sub_trajectory_ = subscribe<Trajectory>(
    "input/trajectory", 1,
    &PlanningValidatorNode::onTrajectory, this);

  ros::NodeHandle scene_node(*this);
  scene_node.setCallbackQueue(&static_scene_queue_);
  sub_static_scene_ = scene_node.subscribe<autoware_planning_msgs::StaticObstacleScene>(
    "/planning/scenario_planning/lane_driving/behavior_planning/static_obstacle_scene", 10,
    &PlanningValidatorNode::onStaticScene, this);
  static_scene_timer_ = createWallTimer(ros::WallDuration(0.01),
    [this](const ros::WallTimerEvent &) {
      const auto previous_scene = static_scene_;
      static_scene_queue_.callAvailable();
      if (static_scene_ == previous_scene) return;
      if (last_published_trajectory_) {
        auto checked = *last_published_trajectory_;
        if (guardStaticScene(checked)) publishSceneCheckedTrajectory(std::move(checked));
      }
    });

  // publishers
  pub_traj_ = create_publisher<Trajectory>("output/trajectory", 1);
  pub_status_ = create_publisher<PlanningValidatorStatus>("output/validation_status", 1);
  pub_markers_ = create_publisher<visualization_msgs::MarkerArray>("output/markers", 1);
  pub_scene_check_ = create_publisher<diagnostic_msgs::DiagnosticArray>("debug/static_scene_check", 10);
  pub_processing_time_ms_ = create_publisher<Float64Stamped>("debug/processing_time_ms", 1);

  context_ = std::make_shared<PlanningValidatorContext>(this);
  context_->set_diag_id("planning_validator");

  setupParameters();

  // Initialize Manager
  for (const auto & name : declare_parameter<std::vector<std::string>>("launch_modules")) {
    // workaround: Since ROS 2 can't get empty list, launcher set [''] on the parameter.
    if (name.empty()) {
      break;
    }
    manager_.load_plugin(*this, name, context_);
  }

  logger_configure_ = std::make_unique<autoware_utils::LoggerLevelConfigure>(*this);
  published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(debug_context_);
}

void PlanningValidatorNode::setupParameters()
{
  auto & p = context_->params;

  const auto value = declare_parameter<int>("default_handling_type");
  try {
    p.default_handling_type = get_handling_type(value);
  } catch (const std::exception & e) {
    throw std::invalid_argument(e.what());
  }

  p.publish_diag = declare_parameter<bool>("publish_diag");
  p.diag_error_count_threshold = declare_parameter<int>("diag_error_count_threshold");
  p.display_on_terminal = declare_parameter<bool>("display_on_terminal");

  p.soft_stop_deceleration = declare_parameter<double>("soft_stop_deceleration");
  p.soft_stop_jerk_lim = declare_parameter<double>("soft_stop_jerk_lim");
  static_scene_deceleration_ = declare_parameter<double>("static_scene_deceleration");
  if (!std::isfinite(static_scene_deceleration_) || static_scene_deceleration_ <= 0.0) {
    throw std::invalid_argument("static_scene_deceleration must be finite and positive");
  }
  p.th_traffic_light_timeout = declare_parameter<double>("th_traffic_light_timeout");
  p.require_obstacle_pointcloud = declare_parameter<bool>("require_obstacle_pointcloud");
  context_->data->require_obstacle_pointcloud = p.require_obstacle_pointcloud;
}

bool PlanningValidatorNode::isDataReady()
{
  const auto waiting = [this](const auto s) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for %s", s.c_str());
    return false;
  };

  std::string msg;
  if (!context_->data->is_ready(msg)) {
    return waiting(msg);
  }
  return true;
}

void PlanningValidatorNode::setData(const Trajectory::ConstPtr & traj_msg)
{
  auto & data = context_->data;
  data->current_kinematics = sub_kinematics_.take_data();
  data->current_acceleration = sub_acceleration_.take_data();
  data->obstacle_pointcloud = sub_pointcloud_.take_data();
  data->traffic_signals = sub_traffic_signals_.take_data();
  data->set_current_trajectory(traj_msg);
  data->set_route(sub_route_.take_data());
  data->set_map(sub_lanelet_map_bin_.take_data());
}

void PlanningValidatorNode::onTrajectory(const Trajectory::ConstPtr & traj_msg)
{
  stop_watch_.tic(__func__);

  setData(traj_msg);

  if (!isDataReady()) return;

  // Check operational mode state
  OperationModeState::ConstPtr operation_mode_msg = sub_operational_state_.take_data();
  if (operation_mode_msg) {
    flag_autonomous_control_enabled_ = infer_autonomous_control_state(operation_mode_msg);
  }

  context_->init_validation_status();
  context_->reset_handling();

  context_->data->set_nearest_trajectory_indices();

  context_->debug_pose_publisher->clearMarkers();

  manager_.validate();

  auto & s = context_->validation_status;

  if (!flag_autonomous_control_enabled_) {
    // if warnings or errors are being suppressed, printing simple logs
    if (!isAllValid(*s)) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 3000, "Suppressing planning validation during manual driving");
    }
    context_->init_validation_status();
  }

  s->invalid_count = isAllValid(*s) ? 0 : s->invalid_count + 1;

  context_->update_diag();

  publishTrajectory();

  // for debug
  publishProcessingTime(stop_watch_.toc(__func__));
  publishDebugInfo();
  displayStatus();
}

bool PlanningValidatorNode::isAllValid(const PlanningValidatorStatus & s) const
{
  return s.is_valid_size && s.is_valid_finite_value && s.is_valid_interval &&
         s.is_valid_relative_angle && s.is_valid_curvature && s.is_valid_lateral_acc &&
         s.is_valid_lateral_jerk && s.is_valid_longitudinal_max_acc &&
         s.is_valid_longitudinal_min_acc && s.is_valid_steering && s.is_valid_steering_rate &&
         s.is_valid_velocity_deviation && s.is_valid_distance_deviation &&
         s.is_valid_longitudinal_distance_deviation && s.is_valid_forward_trajectory_length &&
         s.is_valid_latency && s.is_valid_yaw_deviation && s.is_valid_trajectory_shift &&
         s.is_valid_intersection_collision_check && s.is_valid_rear_collision_check;
}

void PlanningValidatorNode::publishTrajectory()
{
  const auto & status = *context_->validation_status;
  const auto & data = context_->data;
  // Validation check is all green. Publish the trajectory.
  if (isAllValid(status)) {
    publishSceneCheckedTrajectory(*data->current_trajectory);
    published_time_publisher_->publish_if_subscribed(
      pub_traj_.identity(), data->current_trajectory->header.stamp);
    data->last_valid_trajectory = data->current_trajectory;
    soft_stop_trajectory_ = nullptr;
    return;
  }

  const auto & params = context_->params;

  // An upstream emergency stop must never be replaced by a positive-speed
  // historical trajectory merely because its geometry failed a continuity check.
  const bool stop_requested = !data->current_trajectory->points.empty() &&
    std::all_of(data->current_trajectory->points.begin(), data->current_trajectory->points.end(),
      [](const auto & p) { return std::abs(p.longitudinal_velocity_mps) < 1e-4; });
  if (stop_requested) {
    auto stopped = status.is_valid_finite_value && status.is_valid_size
      ? *data->current_trajectory
      : (data->last_valid_trajectory ? *data->last_valid_trajectory : *data->current_trajectory);
    if ((!status.is_valid_finite_value || !status.is_valid_size) && !data->last_valid_trajectory) return;
    for (auto & point : stopped.points) point.longitudinal_velocity_mps = 0.0;
    stopped.header = data->current_trajectory->header;
    publishSceneCheckedTrajectory(stopped);
    published_time_publisher_->publish_if_subscribed(pub_traj_.identity(), stopped.header.stamp);
    soft_stop_trajectory_ = nullptr;
    return;
  }

  //  ----- invalid factor is found. Publish previous trajectory. -----

  const auto handling_type = context_->get_handling();

  if (handling_type == InvalidTrajectoryHandlingType::PUBLISH_AS_IT_IS) {
    publishSceneCheckedTrajectory(*data->current_trajectory);
    published_time_publisher_->publish_if_subscribed(
      pub_traj_.identity(), data->current_trajectory->header.stamp);
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000, "Caution! Invalid Trajectory published.");
    return;
  }

  if (!data->last_valid_trajectory) {
    // trajectory is not published.
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Invalid Trajectory detected, no valid trajectory found in the past. Trajectory is not "
      "published.");
    return;
  }

  if (handling_type == InvalidTrajectoryHandlingType::USE_PREVIOUS_RESULT) {
    const auto & pub_trajectory = *data->last_valid_trajectory;
    publishSceneCheckedTrajectory(pub_trajectory);
    published_time_publisher_->publish_if_subscribed(pub_traj_.identity(), pub_trajectory.header.stamp);
    RCLCPP_ERROR(get_logger(), "Invalid Trajectory detected. Use previous trajectory.");
    return;
  }

  if (handling_type == InvalidTrajectoryHandlingType::USE_PREVIOUS_RESULT_WITH_SOFT_STOP) {
    if (!soft_stop_trajectory_) {
      const auto nearest_idx = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
        data->last_valid_trajectory->points, data->current_kinematics->pose.pose);
      soft_stop_trajectory_ = boost::make_shared<Trajectory>(full_source::getStopTrajectory(
        *data->last_valid_trajectory, nearest_idx, data->current_kinematics->twist.twist.linear.x,
        data->current_acceleration->accel.accel.linear.x, params.soft_stop_deceleration,
        params.soft_stop_jerk_lim));
    }
    const auto & pub_trajectory = *soft_stop_trajectory_;
    publishSceneCheckedTrajectory(pub_trajectory);
    published_time_publisher_->publish_if_subscribed(pub_traj_.identity(), pub_trajectory.header.stamp);
    RCLCPP_ERROR(get_logger(), "Invalid Trajectory detected. Use soft stop trajectory.");
  }
}

void PlanningValidatorNode::onStaticScene(
  const autoware_planning_msgs::StaticObstacleScene::ConstPtr & scene)
{
  if (static_scene_ && scene->header.stamp < static_scene_->header.stamp) {
    last_published_trajectory_.reset();
    soft_stop_trajectory_.reset();
    context_->data->last_valid_trajectory.reset();
  }
  static_scene_ = scene;
  // The dedicated queue is drained by the wall timer and before publishing.
  // Only update state here to avoid recursive publication while draining it.
}

bool PlanningValidatorNode::guardStaticScene(Trajectory & trajectory)
{
  scene_check_odometry_ = sub_kinematics_.take_data();
  if (!static_scene_ || static_scene_->obstacles.empty() || trajectory.points.empty()) return false;
  const auto odometry = scene_check_odometry_;
  if (!odometry) return false;
  auto & points = trajectory.points;
  const size_t ego = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    points, odometry->pose.pose);
  const bool frame_mismatch = trajectory.header.frame_id != static_scene_->header.frame_id;
  TrajectoryPoint measured;
  measured.pose = odometry->pose.pose;
  const auto & vehicle = context_->vehicle_info;
  const bool measured_overlap = autoware::motion_utils::firstObstacleCollision(
    {measured}, 0, static_scene_->obstacles, vehicle).has_value();
  const auto collision = autoware::motion_utils::firstObstacleCollision(
    points, ego, static_scene_->obstacles, vehicle);
  if (!frame_mismatch && !measured_overlap && !collision) return false;
  const size_t unsafe = collision.value_or(ego);
  if (!frame_mismatch && !measured_overlap) {
    for (size_t i = ego; i <= unsafe; ++i) {
      if (std::abs(points[i].longitudinal_velocity_mps) < 1e-4) return false;
    }
  }
  double clear = 0.0;
  for (size_t i = ego + 1; i <= unsafe; ++i) {
    clear += std::hypot(points[i].pose.position.x - points[i - 1].pose.position.x,
                        points[i].pose.position.y - points[i - 1].pose.position.y);
  }
  const double deceleration = static_scene_deceleration_;
  const double stop_distance = std::max(0.0, clear - 1.0);
  const double speed = odometry->twist.twist.linear.x;
  const bool emergency = frame_mismatch || measured_overlap ||
    speed * speed > 2.0 * deceleration * stop_distance;
  bool changed = false;
  double distance = 0.0;
  for (size_t i = 0; i < points.size(); ++i) {
    if (i > ego) {
      distance += std::hypot(points[i].pose.position.x - points[i - 1].pose.position.x,
                             points[i].pose.position.y - points[i - 1].pose.position.y);
    }
    if (!emergency && i < ego) continue;
    const double limit = emergency ? 0.0 :
      std::sqrt(2.0 * deceleration * std::max(0.0, stop_distance - distance));
    const auto previous = points[i].longitudinal_velocity_mps;
    points[i].longitudinal_velocity_mps = emergency ? 0.0 : std::min<double>(previous, limit);
    if (points[i].longitudinal_velocity_mps < previous - 1e-4) {
      points[i].acceleration_mps2 = limit > 0.0 ?
        std::min<double>(points[i].acceleration_mps2, -deceleration) : 0.0;
    }
    changed = changed || points[i].longitudinal_velocity_mps < previous - 1e-4;
  }
  if (changed) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "[Static scene] Newest obstacle invalidates output; braking on same geometry "
      "(clear=%.2f emergency=%d)", clear, static_cast<int>(emergency));
  }
  return changed;
}

void PlanningValidatorNode::publishSceneCheckedTrajectory(Trajectory trajectory)
{
  // A scene can arrive during the footprint check itself. Consume it and repeat
  // before committing the command. All callbacks run on the single ROS spinner.
  autoware_planning_msgs::StaticObstacleScene::ConstPtr checked_scene;
  do {
    static_scene_queue_.callAvailable();
    checked_scene = static_scene_;
    guardStaticScene(trajectory);
    static_scene_queue_.callAvailable();
  } while (checked_scene != static_scene_);

  // Record the exact scene and ego used for this command. An independent recorder
  // can receive two topics in a different order and cannot infer this snapshot.
  trajectory.header.seq = 0;  // Normalize the ROS transport sequence for the checksum.
  const auto serialized = ros::serialization::serializeMessage(trajectory);
  boost::crc_32_type checksum;
  checksum.process_bytes(serialized.message_start, serialized.num_bytes - 4);
  diagnostic_msgs::DiagnosticArray evidence;
  evidence.header.stamp = now();
  diagnostic_msgs::DiagnosticStatus status;
  status.name = "static_scene_check";
  const auto add = [&](const std::string & key, const auto & value) {
    diagnostic_msgs::KeyValue entry;
    entry.key = key;
    std::ostringstream text;
    text << std::setprecision(17) << value;
    entry.value = text.str();
    status.values.push_back(std::move(entry));
  };
  add("publication_index", published_trajectory_sequence_++);
  add("trajectory_crc32", checksum.checksum());
  add("scene_stamp_ns", checked_scene ? checked_scene->header.stamp.toNSec() : 0);
  add("validated_wall_ns", std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count());
  if (scene_check_odometry_) {
    add("ego_x", scene_check_odometry_->pose.pose.position.x);
    add("ego_y", scene_check_odometry_->pose.pose.position.y);
    add("ego_yaw", tf2::getYaw(scene_check_odometry_->pose.pose.orientation));
    add("ego_v", scene_check_odometry_->twist.twist.linear.x);
  }
  evidence.status.push_back(std::move(status));
  pub_traj_->publish(trajectory);
  pub_scene_check_->publish(evidence);
  last_published_trajectory_ = boost::make_shared<Trajectory>(std::move(trajectory));
}

void PlanningValidatorNode::publishProcessingTime(const double processing_time_ms)
{
  Float64Stamped msg{};
  msg.stamp = validator_native::stamp(this->now());
  msg.data = processing_time_ms;
  pub_processing_time_ms_->publish(msg);
}

void PlanningValidatorNode::publishDebugInfo()
{
  const auto & status = context_->validation_status;
  status->stamp = get_clock()->now();
  pub_status_->publish(*status);

  const auto & data = context_->data;

  if (!isAllValid(*status)) {
    geometry_msgs::Pose front_pose = data->current_kinematics->pose.pose;
    shiftPose(
      front_pose, context_->vehicle_info.front_overhang_m + context_->vehicle_info.wheel_base_m);
    auto offset_pose = front_pose;
    shiftPose(offset_pose, 0.5);
    context_->debug_pose_publisher->pushVirtualWall(front_pose);
    const auto status_debug_str = context_->debug_pose_publisher->getStatusDebugString(*status);
    context_->debug_pose_publisher->pushWarningMsg(
      offset_pose, "INVALID PLANNING" + status_debug_str);
  }
  context_->debug_pose_publisher->publish();
}

void PlanningValidatorNode::displayStatus()
{
  if (!context_->params.display_on_terminal) return;

  const auto warn = [this](const bool status, const std::string & msg) {
    if (!status) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "%s", msg.c_str());
    }
  };

  const auto & s = context_->validation_status;

  warn(s->is_valid_size, "planning trajectory size is invalid, too small.");
  warn(s->is_valid_curvature, "planning trajectory curvature is too large!!");
  warn(s->is_valid_finite_value, "planning trajectory has invalid value!!");
  warn(s->is_valid_interval, "planning trajectory interval is too long!!");
  warn(s->is_valid_lateral_acc, "planning trajectory lateral acceleration is too high!!");
  warn(s->is_valid_lateral_jerk, "planning trajectory lateral jerk is too high!!");
  warn(s->is_valid_longitudinal_max_acc, "planning trajectory acceleration is too high!!");
  warn(s->is_valid_longitudinal_min_acc, "planning trajectory deceleration is too high!!");
  warn(s->is_valid_relative_angle, "planning trajectory yaw angle varies too fast!!");
  warn(s->is_valid_steering, "planning trajectory expected steering angle is too high!!");
  warn(s->is_valid_steering_rate, "planning trajectory expected steering angle rate is too high!!");
  warn(s->is_valid_velocity_deviation, "planning trajectory velocity deviation is too high!!");
  warn(s->is_valid_distance_deviation, "planning trajectory is too far from ego!!");
  warn(
    s->is_valid_longitudinal_distance_deviation,
    "planning trajectory is too far from ego in longitudinal direction!!");
  warn(s->is_valid_forward_trajectory_length, "planning trajectory forward length is not enough!!");
  warn(s->is_valid_latency, "planning component latency is larger than threshold!!");
  warn(s->is_valid_yaw_deviation, "planning trajectory yaw difference from ego yaw is too large!!");
  warn(s->is_valid_trajectory_shift, "planning trajectory had sudden shift!!");
  warn(
    s->is_valid_intersection_collision_check,
    "planning trajectory leads to collision!! (intersection objects)");
  warn(s->is_valid_rear_collision_check, "planning trajectory leads to collision!! (rear objects)");
}

bool PlanningValidatorNode::infer_autonomous_control_state(
  const OperationModeState::ConstPtr msg)
{
  return (msg->mode == OperationModeState::AUTONOMOUS) && (msg->is_autoware_control_enabled);
}

}  // namespace autoware::planning_validator::full_source
