#include "autoware/behavior_velocity_planner/node.hpp"
#include "smoother_initialization.hpp"
#include "autoware/behavior_velocity_planner/pointcloud_transform.hpp"

#include <autoware/route_handler/route_handler.hpp>


#include <utility>

namespace autoware::behavior_velocity_planner
{
namespace
{
autoware_planning_msgs::Path toPath(const autoware_internal_planning_msgs::PathWithLaneId & path)
{
  autoware_planning_msgs::Path output;
  output.header = path.header;
  output.left_bound = path.left_bound;
  output.right_bound = path.right_bound;
  output.points.reserve(path.points.size());
  for (const auto & point : path.points) {
    output.points.push_back(point.point);
  }
  return output;
}
}  // namespace

BehaviorVelocityPlannerNode::BehaviorVelocityPlannerNode(const ros::NodeHandle & node)
: node_(node), private_node_("~"), tf_buffer_(ros::Duration(10.0)), tf_listener_(tf_buffer_),
  planner_data_(private_node_)
{
  // Original onParam initializes the analytical smoother once for all modules.
  planner_data_.velocity_smoother_ = makeBvpVelocitySmoother(
    private_node_, planner_data_.vehicle_info_.wheel_base_m);
  trigger_sub_path_with_lane_id_ = private_node_.subscribe(
    "input/path_with_lane_id", 1, &BehaviorVelocityPlannerNode::onTrigger, this);
  path_pub_ = private_node_.advertise<autoware_planning_msgs::Path>("output/path", 1);
  odometry_sub_ = private_node_.subscribe(
    "input/vehicle_odometry", 1, &BehaviorVelocityPlannerNode::onOdometry, this);
  acceleration_sub_ = private_node_.subscribe(
    "input/accel", 1, &BehaviorVelocityPlannerNode::onAcceleration, this);
  predicted_objects_sub_ = private_node_.subscribe(
    "input/dynamic_objects", 1, &BehaviorVelocityPlannerNode::onPredictedObjects, this);
  occupancy_grid_sub_ = private_node_.subscribe(
    "input/occupancy_grid", 1, &BehaviorVelocityPlannerNode::onOccupancyGrid, this);
  external_velocity_limit_sub_ = private_node_.subscribe(
    "input/external_velocity_limit_mps", 1,
    &BehaviorVelocityPlannerNode::onExternalVelocityLimit, this);
  traffic_signals_sub_ = private_node_.subscribe(
    "input/traffic_signals", 1, &BehaviorVelocityPlannerNode::onTrafficSignals, this);
  lanelet_map_sub_ = private_node_.subscribe(
    "input/vector_map", 1, &BehaviorVelocityPlannerNode::onLaneletMap, this);
  no_ground_pointcloud_sub_ = private_node_.subscribe(
    "input/no_ground_pointcloud", 1, &BehaviorVelocityPlannerNode::onNoGroundPointCloud, this);

  std::vector<std::string> launch_modules;
  private_node_.param("launch_modules", launch_modules, std::vector<std::string>{});
  for (const auto & name : launch_modules) {
    if (!name.empty()) {
      planner_manager_.launchScenePlugin(private_node_, name);
    }
  }
}

void BehaviorVelocityPlannerNode::onOdometry(const nav_msgs::Odometry::ConstPtr & odometry)
{
  if (!odometry) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  planner_data_.updateOdometry(*odometry);
}

void BehaviorVelocityPlannerNode::onAcceleration(
  const geometry_msgs::AccelWithCovarianceStamped::ConstPtr & acceleration)
{
  if (!acceleration) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  planner_data_.current_acceleration =
    std::make_shared<geometry_msgs::AccelWithCovarianceStamped>(*acceleration);
}

void BehaviorVelocityPlannerNode::onPredictedObjects(
  const autoware_perception_msgs::PredictedObjects::ConstPtr & objects)
{
  if (!objects) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  planner_data_.predicted_objects =
    std::make_shared<autoware_perception_msgs::PredictedObjects>(*objects);
}

void BehaviorVelocityPlannerNode::onOccupancyGrid(
  const nav_msgs::OccupancyGrid::ConstPtr & occupancy_grid)
{
  if (!occupancy_grid) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  planner_data_.occupancy_grid = std::make_shared<nav_msgs::OccupancyGrid>(*occupancy_grid);
}

void BehaviorVelocityPlannerNode::onExternalVelocityLimit(
  const autoware_internal_planning_msgs::VelocityLimit::ConstPtr & velocity_limit)
{
  if (!velocity_limit) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  planner_data_.external_velocity_limit = *velocity_limit;
}

void BehaviorVelocityPlannerNode::onTrafficSignals(
  const autoware_perception_msgs::TrafficLightGroupArray::ConstPtr & traffic_signals)
{
  if (!traffic_signals) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  planner_data_.updateTrafficSignals(*traffic_signals);
}

void BehaviorVelocityPlannerNode::onLaneletMap(const autoware_map_msgs::LaneletMapBin::ConstPtr & map)
{
  if (!map) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  planner_data_.route_handler_ = std::make_shared<autoware::route_handler::RouteHandler>(*map);
}

void BehaviorVelocityPlannerNode::onNoGroundPointCloud(
  const sensor_msgs::PointCloud2::ConstPtr & pointcloud)
{
  if (!pointcloud) {
    return;
  }
  geometry_msgs::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(
      "map", pointcloud->header.frame_id, pointcloud->header.stamp, ros::Duration(0.1));
  } catch (const tf2::TransformException & error) {
    ROS_WARN_STREAM_NAMED(
      private_node_.getNamespace(),
      "no transform found for no_ground_pointcloud: " << error.what());
    return;
  }

  const auto transformed = transformNoGroundPointCloud(*pointcloud, transform);
  std::lock_guard<std::mutex> lock(mutex_);
  planner_data_.no_ground_pointcloud = transformed;
}

void BehaviorVelocityPlannerNode::onTrigger(
  const autoware_internal_planning_msgs::PathWithLaneId::ConstPtr & input_path)
{
  if (!input_path || input_path->points.empty()) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  const auto planned_path = planner_manager_.planPathVelocity(
    std::make_shared<const PlannerData>(planner_data_), *input_path);
  path_pub_.publish(toPath(planned_path));
}
}  // namespace autoware::behavior_velocity_planner
