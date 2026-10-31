// Copyright 2025 Tier IV, Inc.
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

#include "autoware/behavior_velocity_planner/experimental/full_node.hpp"

#include <autoware/trajectory/utils/find_intervals.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>
#include "../smoother_initialization.hpp"
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_utils_pcl/transforms.hpp>
#include <autoware_utils_rclcpp/parameter.hpp>
#include <tf2/utils.h>
#include <autoware_utils_math/normalization.hpp>
#include <autoware_utils_math/constants.hpp>
#include <tf2_eigen/tf2_eigen.h>

#include <lanelet2_routing/Route.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <std_msgs/Bool.h>
#include <autoware/behavior_velocity_planner/recovery_stop_context.hpp>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{

namespace
{

bool is_driving_forward(const Trajectory & trajectory)
{
  const auto start_yaw = tf2::getYaw(trajectory.compute(0).point.pose.orientation);
  const auto azimuth = trajectory.azimuth(0);
  return std::abs(autoware_utils_math::normalize_radian(start_yaw - azimuth)) <
         autoware_utils_math::pi / 2.0;
}

}  // namespace

FullBehaviorVelocityPlannerNode::FullBehaviorVelocityPlannerNode(ros::NodeHandle private_node)
: bvp_full_native::Node(std::move(private_node)),
  tf_buffer_(ros::Duration(10.0)),
  tf_listener_(tf_buffer_),
  planner_data_(*this)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  // Trigger Subscriber
  trigger_sub_path_with_lane_id_ = subscribe("input/path_with_lane_id", 1, &FullBehaviorVelocityPlannerNode::onTrigger, this);

  srv_load_plugin_ = advertiseService("service/load_plugin", &FullBehaviorVelocityPlannerNode::onLoadPlugin, this);
  srv_unload_plugin_ = advertiseService("service/unload_plugin", &FullBehaviorVelocityPlannerNode::onUnloadPlugin, this);

  // set velocity smoother param
  onParam();

  // Publishers
  recovery_stop_pub_ = this->advertise<std_msgs::Bool>("/planning/recovery/behavior_stop",1);
  path_pub_ = this->create_publisher<autoware_planning_msgs::Path>("output/path", 1);
  debug_viz_pub_ = this->create_publisher<visualization_msgs::MarkerArray>("debug/path", 1);
  processing_time_publisher_ =
    this->create_publisher<autoware_internal_debug_msgs::Float64Stamped>(
      "debug/processing_time_ms", 1);

  // Parameters
  forward_path_length_ = declare_parameter<double>("forward_path_length");
  backward_path_length_ = declare_parameter<double>("backward_path_length");
  behavior_output_path_interval_ = declare_parameter<double>("behavior_output_path_interval");
  use_occupancy_grid_ = declare_parameter<bool>("use_occupancy_grid", true);

  // nearest search
  planner_data_.ego_nearest_dist_threshold =
    declare_parameter<double>("ego_nearest_dist_threshold");
  planner_data_.ego_nearest_yaw_threshold = declare_parameter<double>("ego_nearest_yaw_threshold");

  // is simulation or not
  planner_data_.is_simulation = declare_parameter<bool>("is_simulation");

  // Initialize PlannerManager
  for (const auto & name : declare_parameter<std::vector<std::string>>("launch_modules")) {
    // workaround: Since ROS 2 can't get empty list, launcher set [''] on the parameter.
    if (name == "") {
      break;
    }
    if (!planner_manager_.isPluginAvailable(name)) throw std::runtime_error("Required BVP plugin unavailable: " + name);
    planner_manager_.launchScenePlugin(*this, name);
  }

  logger_configure_ = std::make_unique<autoware_utils_logging::LoggerLevelConfigure>(*this);
  published_time_publisher_ = std::make_unique<autoware_utils_debug::PublishedTimePublisher>(debug_context_);
}

bool FullBehaviorVelocityPlannerNode::onLoadPlugin(LoadPlugin::Request & request, LoadPlugin::Response & response)
{
  std::unique_lock<std::mutex> lock(mutex_);
  response.success=false;
  if (!planner_manager_.isPluginAvailable(request.plugin_name)) return true;
  try { planner_manager_.launchScenePlugin(*this,request.plugin_name); response.success=true; }
  catch(const std::exception & error) { ROS_ERROR("BVP plugin service failed: %s",error.what()); }
  return true;
}

bool FullBehaviorVelocityPlannerNode::onUnloadPlugin(UnloadPlugin::Request & request, UnloadPlugin::Response & response)
{
  std::unique_lock<std::mutex> lock(mutex_);
  response.success=false;
  if (!planner_manager_.hasModule(request.plugin_name)) return true;
  try { planner_manager_.removeScenePlugin(*this,request.plugin_name); response.success=true; }
  catch(const std::exception & error) { ROS_ERROR("BVP plugin service failed: %s",error.what()); }
  return true;
}

void FullBehaviorVelocityPlannerNode::onParam()
{
  // Note(VRichardJP): mutex lock is not necessary as onParam is only called once in the
  // constructed. It would be required if it was a callback. std::lock_guard<std::mutex>
  // lock(mutex_);
  planner_data_.velocity_smoother_ = makeBvpVelocitySmoother(*this, planner_data_.vehicle_info_.wheel_base_m);
}

bool FullBehaviorVelocityPlannerNode::processNoGroundPointCloud(
  const sensor_msgs::PointCloud2::ConstPtr msg)
{
  geometry_msgs::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(
      "map", msg->header.frame_id, msg->header.stamp, ros::Duration(0.1));
  } catch (tf2::TransformException & e) {
    RCLCPP_WARN(get_logger(), "no transform found for no_ground_pointcloud: %s", e.what());
    return false;
  }

  pcl::PointCloud<pcl::PointXYZ> pc;
  pcl::fromROSMsg(*msg, pc);

  Eigen::Affine3f affine = tf2::transformToEigen(transform.transform).cast<float>();
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_transformed(new pcl::PointCloud<pcl::PointXYZ>);

  if (!pc.empty()) {
    autoware_utils_pcl::transform_pointcloud(pc, *pc_transformed, affine);
  }

  planner_data_.no_ground_pointcloud = pc_transformed;
  return true;
}

void FullBehaviorVelocityPlannerNode::processOdometry(const nav_msgs::Odometry::ConstPtr msg)
{
  auto current_odometry = std::make_shared<geometry_msgs::PoseStamped>();
  current_odometry->header = msg->header;
  current_odometry->pose = msg->pose.pose;
  planner_data_.current_odometry = current_odometry;

  auto current_velocity = std::make_shared<geometry_msgs::TwistStamped>();
  current_velocity->header = msg->header;
  current_velocity->twist = msg->twist.twist;
  planner_data_.current_velocity = current_velocity;

  // Add velocity to buffer
  planner_data_.velocity_buffer.push_front(*current_velocity);
  const ros::Time now = this->now();
  while (!planner_data_.velocity_buffer.empty()) {
    // Check oldest data time
    const auto & s = planner_data_.velocity_buffer.back().header.stamp;
    const auto time_diff =
      now >= s ? now - s : ros::Duration(0);  // Note: negative time throws an exception.

    // Finish when oldest data is newer than threshold
    if (time_diff.toSec() <= PlannerData::velocity_buffer_time_sec) {
      break;
    }

    // Remove old data
    planner_data_.velocity_buffer.pop_back();
  }
}

void FullBehaviorVelocityPlannerNode::processTrafficSignals(
  const autoware_perception_msgs::TrafficLightGroupArray::ConstPtr msg)
{
  // clear previous observation
  planner_data_.traffic_light_id_map_raw_.clear();
  const auto traffic_light_id_map_last_observed_old =
    planner_data_.traffic_light_id_map_last_observed_;
  planner_data_.traffic_light_id_map_last_observed_.clear();
  for (const auto & signal : msg->traffic_light_groups) {
    TrafficSignalStamped traffic_signal;
    traffic_signal.stamp = bvp_full_native::time(msg->stamp);
    traffic_signal.signal = signal;
    planner_data_.traffic_light_id_map_raw_[signal.traffic_light_group_id] = traffic_signal;
    const bool is_unknown_observation =
      std::any_of(signal.elements.begin(), signal.elements.end(), [](const auto & element) {
        return element.color == autoware_perception_msgs::TrafficLightElement::UNKNOWN;
      });
    // if the observation is UNKNOWN and past observation is available, only update the timestamp
    // and keep the body of the info
    const auto old_data =
      traffic_light_id_map_last_observed_old.find(signal.traffic_light_group_id);
    if (is_unknown_observation && old_data != traffic_light_id_map_last_observed_old.end()) {
      // copy last observation
      planner_data_.traffic_light_id_map_last_observed_[signal.traffic_light_group_id] =
        old_data->second;
      // update timestamp
      planner_data_.traffic_light_id_map_last_observed_[signal.traffic_light_group_id].stamp =
        bvp_full_native::time(msg->stamp);
    } else {
      // if (1)the observation is not UNKNOWN or (2)the very first observation is UNKNOWN
      planner_data_.traffic_light_id_map_last_observed_[signal.traffic_light_group_id] =
        traffic_signal;
    }
  }
}

bool FullBehaviorVelocityPlannerNode::processData(bvp_full_native::Clock clock)
{
  bool is_ready = true;
  const auto & logData = [&clock, this](const std::string & data_type) {
    std::string msg = "Waiting for " + data_type + " data";
    RCLCPP_INFO_THROTTLE(get_logger(), clock, logger_throttle_interval, "%s", msg.c_str());
  };

  const auto & getData = [&logData](
                           auto & dest, auto & sub, const std::string & data_type = "",
                           const bool is_required = true) {
    if (!is_required) {
      return true;
    }

    const auto temp = sub.take_data();
    if (temp) {
      bvp_full_native::assign_message(dest, temp);
      return true;
    }
    if (!data_type.empty()) logData(data_type);
    return false;
  };

  const auto required_subscriptions = planner_manager_.getRequiredSubscriptions();

  is_ready &= getData(planner_data_.current_acceleration, sub_acceleration_, "acceleration");
  is_ready &= getData(
    planner_data_.predicted_objects, sub_predicted_objects_, "predicted_objects",
    required_subscriptions.predicted_objects);
  is_ready &= getData(
    planner_data_.occupancy_grid, sub_occupancy_grid_, "occupancy_grid",
    use_occupancy_grid_ && required_subscriptions.occupancy_grid_map);

  nav_msgs::Odometry::ConstPtr odometry;
  is_ready &= getData(odometry, sub_vehicle_odometry_, "odometry");
  if (odometry) {
    processOdometry(odometry);
  }

  sensor_msgs::PointCloud2::ConstPtr no_ground_pointcloud;
  is_ready &= getData(
    no_ground_pointcloud, sub_no_ground_pointcloud_, "pointcloud",
    required_subscriptions.no_ground_pointcloud);

  if (required_subscriptions.no_ground_pointcloud && no_ground_pointcloud) {
    is_ready &= processNoGroundPointCloud(no_ground_pointcloud);
  }

  const auto map_data = sub_lanelet_map_.take_data();
  if (map_data) {
    planner_data_.route_handler_ = std::make_shared<route_handler::RouteHandler>(*map_data);
  }

  // planner_data_.external_velocity_limit is std::optional type variable.
  const auto external_velocity_limit = sub_external_velocity_limit_.take_data();
  if (external_velocity_limit) {
    planner_data_.external_velocity_limit = *external_velocity_limit;
  }

  const auto traffic_signals = sub_traffic_signals_.take_data();
  if (traffic_signals) {
    // NOTE: required_subscriptions.traffic_signals is not used since is_ready is not updated here.
    processTrafficSignals(traffic_signals);
  }

  return is_ready;
}

// NOTE: argument planner_data must not be referenced for multithreading
bool FullBehaviorVelocityPlannerNode::isDataReady(bvp_full_native::Clock clock)
{
  if (!planner_data_.velocity_smoother_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), clock, logger_throttle_interval,
      "Waiting for the initialization of velocity smoother");
    return false;
  }

  return processData(clock);
}

void FullBehaviorVelocityPlannerNode::onTrigger(
  const autoware_internal_planning_msgs::PathWithLaneId::ConstPtr input_path_msg)
{
  stop_watch_.tic();
  std::unique_lock<std::mutex> lk(mutex_);

  if (!isDataReady(*get_clock())) {
    return;
  }

  // Load map and check route handler
  if (!planner_data_.route_handler_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), logger_throttle_interval,
      "Waiting for the initialization of route_handler");
    return;
  }

  const auto input_path = autoware::experimental::trajectory::pretty_build(input_path_msg->points);
  if (!input_path) {
    RCLCPP_ERROR(get_logger(), "Failed to convert input path");
    return;
  }

  const auto output_path = generatePath(
    *input_path, input_path_msg->header, input_path_msg->left_bound, input_path_msg->right_bound,
    planner_data_);

  autoware_planning_msgs::Path output_path_msg;
  output_path_msg.header.frame_id = "map";
  output_path_msg.header.stamp = input_path_msg->header.stamp;
  // TODO(someone): This must be updated in each scene module, but copy from input message for now.
  output_path_msg.left_bound = input_path_msg->left_bound;
  output_path_msg.right_bound = input_path_msg->right_bound;

  const auto restored_output = output_path.restore();
  std_msgs::Bool blocked;
  blocked.data = !planner_data_.current_odometry || hasNewBehaviorStop(
    input_path_msg->points, restored_output, planner_data_.current_odometry->pose.position);
  recovery_stop_pub_.publish(blocked);

  for (const auto & p : restored_output) {
    output_path_msg.points.push_back(p.point);
  }

  lk.unlock();

  path_pub_->publish(output_path_msg);
  published_time_publisher_->publish_if_subscribed(path_pub_.identity(), output_path_msg.header.stamp);

  if (debug_viz_pub_->get_subscription_count() > 0) {
    publishDebugMarker(output_path_msg);
  }

  publishProcessingTime();
}

Trajectory FullBehaviorVelocityPlannerNode::generatePath(
  const Trajectory & input_path, const std_msgs::Header & header,
  const std::vector<geometry_msgs::Point> & left_bound,
  const std::vector<geometry_msgs::Point> & right_bound, const PlannerData & planner_data)
{
  // TODO(someone): support backward path
  is_driving_forward_ = is_driving_forward(input_path);
  if (!is_driving_forward_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), logger_throttle_interval,
      "Backward path is NOT supported, just returning input path");
    return input_path;
  }

  // Plan path velocity
  const auto velocity_planned_path =
    planner_manager_.planPathVelocity(planner_data, input_path, header, left_bound, right_bound);

  // check stop point
  constexpr auto stop_threshold = 0.01;  // m/s
  const auto stop_intervals = autoware::experimental::trajectory::find_intervals(
    velocity_planned_path,
    [&](const autoware_internal_planning_msgs::PathPointWithLaneId & p) {
      return std::abs(p.point.longitudinal_velocity_mps) < stop_threshold;
    });

  auto output_path = velocity_planned_path;
  for (const auto & interval : stop_intervals) {
    output_path.longitudinal_velocity_mps().range(interval.start, interval.end).set(0.0);
  }

  return output_path;
}

void FullBehaviorVelocityPlannerNode::publishProcessingTime()
{
  Float64Stamped processing_time_msg;
  processing_time_msg.stamp = bvp_full_native::stamp(get_clock()->now());
  processing_time_msg.data = stop_watch_.toc();
  processing_time_publisher_->publish(processing_time_msg);
}

void FullBehaviorVelocityPlannerNode::publishDebugMarker(const autoware_planning_msgs::Path & path)
{
  visualization_msgs::MarkerArray output_msg;
  for (size_t i = 0; i < path.points.size(); ++i) {
    visualization_msgs::Marker marker;
    marker.header = path.header;
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::Marker::ARROW;
    marker.pose = path.points.at(i).pose;
    marker.scale.y = marker.scale.z = 0.05;
    marker.scale.x = 0.25;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.5);
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    output_msg.markers.push_back(marker);
  }
  debug_viz_pub_->publish(output_msg);
}
}  // namespace autoware::behavior_velocity_planner::experimental
