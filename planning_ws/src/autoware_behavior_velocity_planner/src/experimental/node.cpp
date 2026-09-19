#include "autoware/behavior_velocity_planner/experimental/node.hpp"
#include "../smoother_initialization.hpp"
#include "autoware/behavior_velocity_planner/pointcloud_transform.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>

namespace autoware::behavior_velocity_planner::experimental
{
BehaviorVelocityPlannerNode::BehaviorVelocityPlannerNode(const ros::NodeHandle &)
: tf_buffer_(ros::Duration(10.0)), tf_listener_(tf_buffer_), planner_data_(private_node_)
{
  planner_data_.velocity_smoother_ = makeBvpVelocitySmoother(
    private_node_, planner_data_.vehicle_info_.wheel_base_m);
  path_sub_ = private_node_.subscribe("input/path_with_lane_id", 1, &BehaviorVelocityPlannerNode::onPath, this);
  odometry_sub_ = private_node_.subscribe("input/vehicle_odometry", 1, &BehaviorVelocityPlannerNode::onOdometry, this);
  map_sub_ = private_node_.subscribe("input/vector_map", 1, &BehaviorVelocityPlannerNode::onMap, this);
  acceleration_sub_ = private_node_.subscribe("input/accel", 1, &BehaviorVelocityPlannerNode::onAcceleration, this);
  objects_sub_ = private_node_.subscribe("input/dynamic_objects", 1, &BehaviorVelocityPlannerNode::onObjects, this);
  occupancy_sub_ = private_node_.subscribe("input/occupancy_grid", 1, &BehaviorVelocityPlannerNode::onOccupancy, this);
  traffic_sub_ = private_node_.subscribe("input/traffic_signals", 1, &BehaviorVelocityPlannerNode::onTrafficSignals, this);
  velocity_limit_sub_ = private_node_.subscribe("input/external_velocity_limit_mps", 1, &BehaviorVelocityPlannerNode::onVelocityLimit, this);
  no_ground_pointcloud_sub_ = private_node_.subscribe("input/no_ground_pointcloud", 1, &BehaviorVelocityPlannerNode::onNoGroundPointCloud, this);
  path_pub_ = private_node_.advertise<autoware_planning_msgs::Path>("output/path", 1);

  std::vector<std::string> launch_modules;
  private_node_.param("launch_modules", launch_modules, std::vector<std::string>{});
  for (const auto & name : launch_modules) {
    if (!name.empty() && !planner_manager_.launchScenePlugin(private_node_, name)) {
      ROS_ERROR_STREAM_NAMED(private_node_.getNamespace(), "Failed to load experimental BVP module '" << name << "'");
    }
  }
}

void BehaviorVelocityPlannerNode::onOdometry(const nav_msgs::Odometry::ConstPtr & message)
{
  if (!message) return;
  std::lock_guard<std::mutex> lock(mutex_);
  planner_data_.updateOdometry(*message);
}

void BehaviorVelocityPlannerNode::onMap(const autoware_map_msgs::LaneletMapBin::ConstPtr & message)
{
  if (!message) return;
  std::lock_guard<std::mutex> lock(mutex_);
  planner_data_.route_handler_ = std::make_shared<autoware::route_handler::RouteHandler>(*message);
}

void BehaviorVelocityPlannerNode::onAcceleration(const geometry_msgs::AccelWithCovarianceStamped::ConstPtr & message)
{
  if (!message) return;
  std::lock_guard<std::mutex> lock(mutex_);
  planner_data_.current_acceleration = std::make_shared<geometry_msgs::AccelWithCovarianceStamped>(*message);
}

void BehaviorVelocityPlannerNode::onObjects(const autoware_perception_msgs::PredictedObjects::ConstPtr & message)
{
  if (!message) return;
  std::lock_guard<std::mutex> lock(mutex_);
  planner_data_.predicted_objects = std::make_shared<autoware_perception_msgs::PredictedObjects>(*message);
}

void BehaviorVelocityPlannerNode::onOccupancy(const nav_msgs::OccupancyGrid::ConstPtr & message)
{
  if (!message) return;
  std::lock_guard<std::mutex> lock(mutex_);
  planner_data_.occupancy_grid = std::make_shared<nav_msgs::OccupancyGrid>(*message);
}

void BehaviorVelocityPlannerNode::onTrafficSignals(
  const autoware_perception_msgs::TrafficLightGroupArray::ConstPtr & message)
{
  if (!message) return;
  std::lock_guard<std::mutex> lock(mutex_);
  planner_data_.updateTrafficSignals(*message);
}

void BehaviorVelocityPlannerNode::onVelocityLimit(
  const autoware_internal_planning_msgs::VelocityLimit::ConstPtr & message)
{
  if (!message) return;
  std::lock_guard<std::mutex> lock(mutex_);
  planner_data_.external_velocity_limit = *message;
}

void BehaviorVelocityPlannerNode::onNoGroundPointCloud(const sensor_msgs::PointCloud2::ConstPtr & message)
{
  if (!message) return;
  geometry_msgs::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform("map", message->header.frame_id, message->header.stamp, ros::Duration(0.1));
  } catch (const tf2::TransformException & error) {
    ROS_WARN_STREAM_NAMED(private_node_.getNamespace(), "no transform found for no_ground_pointcloud: " << error.what());
    return;
  }
  const auto transformed = transformNoGroundPointCloud(*message, transform);
  std::lock_guard<std::mutex> lock(mutex_);
  planner_data_.no_ground_pointcloud = transformed;
}

bool BehaviorVelocityPlannerNode::isDataReady() const
{
  if (!planner_data_.current_odometry || !planner_data_.current_acceleration || !planner_data_.route_handler_) {
    return false;
  }
  const auto required = planner_manager_.getRequiredSubscriptions();
  // Upstream processData explicitly excludes traffic signals from readiness:
  // a missing/empty signal array is handled by each module's signal policy.
  return (!required.predicted_objects || planner_data_.predicted_objects) &&
         (!required.occupancy_grid_map || planner_data_.occupancy_grid) &&
         (!required.no_ground_pointcloud || planner_data_.no_ground_pointcloud);
}

void BehaviorVelocityPlannerNode::onPath(const autoware_internal_planning_msgs::PathWithLaneId::ConstPtr & message)
{
  if (!message || message->points.empty()) return;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!isDataReady()) {
    ROS_WARN_STREAM_THROTTLE(3.0, "experimental BVP input not ready; suppressing path output");
    return;
  }
  const auto input = autoware::experimental::trajectory::pretty_build(message->points);
  if (!input) {
    ROS_ERROR_STREAM_NAMED(private_node_.getNamespace(), "Failed to build experimental BVP trajectory");
    return;
  }
  const auto output = planner_manager_.planPathVelocity(
    planner_data_, *input, message->header, message->left_bound, message->right_bound);
  autoware_planning_msgs::Path result;
  result.header = message->header;
  result.left_bound = message->left_bound;
  result.right_bound = message->right_bound;
  for (const auto & point : output.restore()) result.points.push_back(point.point);
  path_pub_.publish(result);
}
}  // namespace autoware::behavior_velocity_planner::experimental
