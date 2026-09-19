#include <aw149_surround_obstacle_checker/surround_obstacle_checker_ros1_node.hpp>

#include <autoware_perception_msgs/ObjectClassification.h>
#include <builtin_interfaces/Time.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace aw149::surround_obstacle_checker
{
namespace
{
builtin_interfaces::Time rosTimeToBuiltinTime(const ros::Time & time)
{
  builtin_interfaces::Time result;
  result.sec = static_cast<int32_t>(time.sec);
  result.nanosec = time.nsec;
  return result;
}
}  // namespace

SurroundObstacleCheckerRos1Node::SurroundObstacleCheckerRos1Node(
  ros::NodeHandle node, ros::NodeHandle private_node)
: node_(std::move(node)), private_node_(std::move(private_node)),
  parameters_(Parameters::init(private_node_)),
  vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(private_node_).getVehicleInfo()),
  tf_buffer_(), tf_listener_(tf_buffer_)
{
  // ROS2 transient-local publishers map to latched ROS1 publishers.  The output names retain
  // the private-node contract so launch remaps continue to be explicit and testable.
  velocity_limit_pub_ = private_node_.advertise<autoware_internal_planning_msgs::VelocityLimit>(
    "output/max_velocity", 1, true);
  clear_command_pub_ = private_node_.advertise<autoware_internal_planning_msgs::VelocityLimitClearCommand>(
    "output/velocity_limit_clear_command", 1, true);
  processing_time_pub_ = private_node_.advertise<autoware_internal_debug_msgs::Float64Stamped>(
    "debug/processing_time_ms", 1, false);
  debug_marker_pub_ = private_node_.advertise<visualization_msgs::MarkerArray>("debug/marker", 1, false);
  footprint_pub_ = private_node_.advertise<geometry_msgs::PolygonStamped>("debug/footprint", 1, false);
  footprint_offset_pub_ = private_node_.advertise<geometry_msgs::PolygonStamped>("debug/footprint_offset", 1, false);
  footprint_recover_offset_pub_ = private_node_.advertise<geometry_msgs::PolygonStamped>("debug/footprint_recover_offset", 1, false);
  odometry_sub_ = private_node_.subscribe("input/odometry", 1, &SurroundObstacleCheckerRos1Node::onOdometry, this);
  pointcloud_sub_ = private_node_.subscribe("input/pointcloud", 1, &SurroundObstacleCheckerRos1Node::onPointcloud, this);
  objects_sub_ = private_node_.subscribe("input/objects", 1, &SurroundObstacleCheckerRos1Node::onObjects, this);
  planning_factor_interface_ = std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
    &node_, "surround_obstacle_checker");
  timer_ = private_node_.createTimer(ros::Duration(0.1), &SurroundObstacleCheckerRos1Node::onTimer, this);
}

void SurroundObstacleCheckerRos1Node::onOdometry(const nav_msgs::Odometry::ConstPtr & message)
{
  std::lock_guard<std::mutex> lock(input_mutex_); odometry_ = message;
}
void SurroundObstacleCheckerRos1Node::onPointcloud(const sensor_msgs::PointCloud2::ConstPtr & message)
{
  std::lock_guard<std::mutex> lock(input_mutex_); pointcloud_ = message;
}
void SurroundObstacleCheckerRos1Node::onObjects(const autoware_perception_msgs::PredictedObjects::ConstPtr & message)
{
  std::lock_guard<std::mutex> lock(input_mutex_); objects_ = message;
}

double SurroundObstacleCheckerRos1Node::distanceToEgoRectangle(
  const double x, const double y, const ObstacleTypeParameters & type) const
{
  const double front = vehicle_info_.max_longitudinal_offset_m + type.surround_check_front_distance;
  const double rear = -vehicle_info_.rear_overhang_m - type.surround_check_back_distance;
  const double half_width = vehicle_info_.vehicle_width_m * 0.5 + type.surround_check_side_distance;
  const double dx = x < rear ? rear - x : (x > front ? x - front : 0.0);
  const double dy = y < -half_width ? -half_width - y : (y > half_width ? y - half_width : 0.0);
  return std::hypot(dx, dy);
}

std::optional<ObstacleObservation> SurroundObstacleCheckerRos1Node::nearestPointcloud(
  const sensor_msgs::PointCloud2::ConstPtr & pointcloud,
  const nav_msgs::Odometry::ConstPtr & odometry) const
{
  if (!parameters_.obstacle_types.at("pointcloud").enable_check || !pointcloud ||
    !odometry || pointcloud->data.empty()) return std::nullopt;
  try {
    const auto transform = tf_buffer_.lookupTransform("base_link", pointcloud->header.frame_id,
      pointcloud->header.stamp, ros::Duration(0.5));
    pcl::PointCloud<pcl::PointXYZ> input, transformed;
    pcl::fromROSMsg(*pointcloud, input);
    const Eigen::Affine3d affine = tf2::transformToEigen(transform.transform);
    pcl::transformPointCloud(input, transformed, affine.cast<float>());
    ObstacleObservation nearest;
    nearest.is_pointcloud = true;
    double nearest_distance = std::numeric_limits<double>::max();
    for (const auto & point : transformed.points) {
      if (std::isfinite(point.x) && std::isfinite(point.y)) {
        const double distance = distanceToEgoRectangle(
          point.x, point.y, parameters_.obstacle_types.at("pointcloud"));
        if (distance < nearest_distance) {
          nearest_distance = distance;
          nearest.distance = distance;
          const auto & ego = odometry->pose.pose;
          const double yaw = tf2::getYaw(ego.orientation);
          nearest.map_point.x = ego.position.x + std::cos(yaw) * point.x - std::sin(yaw) * point.y;
          nearest.map_point.y = ego.position.y + std::sin(yaw) * point.x + std::cos(yaw) * point.y;
          nearest.map_point.z = ego.position.z + point.z;
        }
      }
    }
    return nearest_distance == std::numeric_limits<double>::max() ? std::nullopt :
      std::optional<ObstacleObservation>{nearest};
  } catch (const tf2::TransformException & exception) {
    ROS_WARN_THROTTLE(5.0, "surround_obstacle_checker skips pointcloud without base_link transform: %s", exception.what());
    return std::nullopt;
  }
}

std::optional<std::string> SurroundObstacleCheckerRos1Node::labelName(const uint8_t label)
{
  using C = autoware_perception_msgs::ObjectClassification;
  switch (label) {
    case C::UNKNOWN: return "unknown"; case C::CAR: return "car"; case C::TRUCK: return "truck";
    case C::BUS: return "bus"; case C::TRAILER: return "trailer"; case C::MOTORCYCLE: return "motorcycle";
    case C::BICYCLE: return "bicycle"; case C::PEDESTRIAN: return "pedestrian"; default: return std::nullopt;
  }
}

std::optional<ObstacleObservation> SurroundObstacleCheckerRos1Node::nearestObject(
  const autoware_perception_msgs::PredictedObjects::ConstPtr & objects,
  const nav_msgs::Odometry::ConstPtr & odometry) const
{
  if (!objects || !odometry) return std::nullopt;
  const auto & ego = odometry->pose.pose;
  const double yaw = tf2::getYaw(ego.orientation);
  const double cos_yaw = std::cos(yaw), sin_yaw = std::sin(yaw);
  namespace bg = boost::geometry;
  using Point2d = bg::model::d2::point_xy<double>;
  using Polygon2d = bg::model::polygon<Point2d>;
  std::optional<ObstacleObservation> nearest;
  for (const auto & object : objects->objects) {
    if (object.classification.empty()) continue;
    const auto label = labelName(object.classification.front().label);
    if (!label || !parameters_.obstacle_types.at(*label).enable_check) continue;
    const auto & position = object.kinematics.initial_pose_with_covariance.pose.position;
    // The source computes polygon-to-polygon distance.  Preserve that contract for both
    // Shape::POLYGON local footprints and the BOUNDING_BOX fallback.
    const double object_yaw = tf2::getYaw(object.kinematics.initial_pose_with_covariance.pose.orientation);
    std::vector<std::pair<double, double>> local_points;
    if (object.shape.type == autoware_perception_msgs::Shape::POLYGON && object.shape.footprint.points.size() >= 3U) {
      for (const auto & point : object.shape.footprint.points) local_points.emplace_back(point.x, point.y);
    } else {
      const double half_length = std::max(0.0, object.shape.dimensions.x) * 0.5;
      const double half_width = std::max(0.0, object.shape.dimensions.y) * 0.5;
      local_points = {{half_length, half_width}, {half_length, -half_width}, {-half_length, -half_width}, {-half_length, half_width}};
    }
    Polygon2d object_polygon;
    for (const auto & corner : local_points) {
      const double map_x = position.x + std::cos(object_yaw) * corner.first - std::sin(object_yaw) * corner.second;
      const double map_y = position.y + std::sin(object_yaw) * corner.first + std::cos(object_yaw) * corner.second;
      const double dx = map_x - ego.position.x, dy = map_y - ego.position.y;
      const double base_x = cos_yaw * dx + sin_yaw * dy;
      const double base_y = -sin_yaw * dx + cos_yaw * dy;
      object_polygon.outer().emplace_back(base_x, base_y);
    }
    bg::correct(object_polygon);
    const auto & type = parameters_.obstacle_types.at(*label);
    Polygon2d ego_polygon;
    const double front = vehicle_info_.max_longitudinal_offset_m + type.surround_check_front_distance;
    const double rear = vehicle_info_.min_longitudinal_offset_m - type.surround_check_back_distance;
    const double left = vehicle_info_.max_lateral_offset_m + type.surround_check_side_distance;
    const double right = vehicle_info_.min_lateral_offset_m - type.surround_check_side_distance;
    ego_polygon.outer().emplace_back(front, left); ego_polygon.outer().emplace_back(front, right);
    ego_polygon.outer().emplace_back(rear, right); ego_polygon.outer().emplace_back(rear, left);
    bg::correct(ego_polygon);
    const double distance = bg::distance(ego_polygon, object_polygon);
    if (!nearest || distance < nearest->distance) {
      ObstacleObservation observation;
      observation.distance = distance;
      observation.is_pointcloud = false;
      observation.map_point = position;
      observation.object_id = object.object_id;
      nearest = observation;
    }
  }
  return nearest;
}

bool SurroundObstacleCheckerRos1Node::stopRequired(
  const bool obstacle_found, const bool vehicle_stopped, const double threshold)
{
  if (!vehicle_stopped) { last_obstacle_found_time_.reset(); return false; }
  if (obstacle_found) { last_obstacle_found_time_ = ros::Time::now(); return true; }
  if (state_ != State::STOP || !last_obstacle_found_time_) return false;
  return (ros::Time::now() - *last_obstacle_found_time_).toSec() <= threshold;
}

geometry_msgs::PolygonStamped SurroundObstacleCheckerRos1Node::footprint(
  const double front_margin, const double side_margin, const double rear_margin) const
{
  geometry_msgs::PolygonStamped polygon; polygon.header.stamp = ros::Time::now(); polygon.header.frame_id = "base_link";
  const std::array<std::pair<double, double>, 5> corners{{
    {vehicle_info_.max_longitudinal_offset_m + front_margin, vehicle_info_.max_lateral_offset_m + side_margin},
    {vehicle_info_.max_longitudinal_offset_m + front_margin, vehicle_info_.min_lateral_offset_m - side_margin},
    {vehicle_info_.min_longitudinal_offset_m - rear_margin, vehicle_info_.min_lateral_offset_m - side_margin},
    {vehicle_info_.min_longitudinal_offset_m - rear_margin, vehicle_info_.max_lateral_offset_m + side_margin},
    {vehicle_info_.max_longitudinal_offset_m + front_margin, vehicle_info_.max_lateral_offset_m + side_margin}}};
  for (const auto & corner : corners) { geometry_msgs::Point32 point; point.x = corner.first; point.y = corner.second; polygon.polygon.points.push_back(point); }
  return polygon;
}

void SurroundObstacleCheckerRos1Node::publishDebug(
  const nav_msgs::Odometry::ConstPtr & odometry,
  const std::optional<ObstacleObservation> & nearest) const
{
  const auto & type = parameters_.obstacle_types.at(parameters_.debug_footprint_label);
  if (parameters_.publish_debug_footprints) {
    footprint_pub_.publish(footprint(0.0, 0.0, 0.0));
    footprint_offset_pub_.publish(footprint(type.surround_check_front_distance, type.surround_check_side_distance, type.surround_check_back_distance));
    footprint_recover_offset_pub_.publish(footprint(type.surround_check_front_distance + parameters_.surround_check_hysteresis_distance,
      type.surround_check_side_distance + parameters_.surround_check_hysteresis_distance,
      type.surround_check_back_distance + parameters_.surround_check_hysteresis_distance));
  }
  visualization_msgs::MarkerArray markers;
  if (state_ == State::STOP && nearest) {
    visualization_msgs::Marker marker; marker.header.stamp = ros::Time::now(); marker.header.frame_id = "map";
    marker.ns = "no_start_obstacle_text"; marker.id = 0; marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD; marker.pose = odometry->pose.pose;
    marker.pose.position = nearest->map_point; marker.pose.position.z += 2.0;
    marker.scale.z = 1.0; marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0; marker.text = "!";
    markers.markers.push_back(marker);
  }
  debug_marker_pub_.publish(markers);
}

void SurroundObstacleCheckerRos1Node::onTimer(const ros::TimerEvent &)
{
  const auto started = std::chrono::steady_clock::now();
  nav_msgs::Odometry::ConstPtr odometry; sensor_msgs::PointCloud2::ConstPtr pointcloud;
  autoware_perception_msgs::PredictedObjects::ConstPtr objects;
  { std::lock_guard<std::mutex> lock(input_mutex_); odometry = odometry_; pointcloud = pointcloud_; objects = objects_; }
  if (!odometry) return;
  const auto pointcloud_obstacle = nearestPointcloud(pointcloud, odometry);
  const auto object_obstacle = nearestObject(objects, odometry);
  std::optional<ObstacleObservation> nearest;
  if (pointcloud_obstacle) nearest = pointcloud_obstacle;
  if (object_obstacle && (!nearest || object_obstacle->distance < nearest->distance)) nearest = object_obstacle;
  const bool vehicle_stopped = std::abs(odometry->twist.twist.linear.x) <= parameters_.stop_state_ego_speed;
  const double boundary = state_ == State::PASS ? 1e-3 : parameters_.surround_check_hysteresis_distance;
  const bool obstacle_found = nearest && nearest->distance < boundary;
  if (state_ == State::PASS && stopRequired(obstacle_found, vehicle_stopped, parameters_.state_clear_time)) {
    state_ = State::STOP;
    autoware_internal_planning_msgs::VelocityLimit limit;
    limit.stamp = rosTimeToBuiltinTime(ros::Time::now()); limit.max_velocity = 0.0F; limit.use_constraints = false;
    limit.sender = "surround_obstacle_checker"; velocity_limit_pub_.publish(limit);
  } else if (state_ == State::STOP && !stopRequired(obstacle_found, vehicle_stopped, parameters_.state_clear_time)) {
    state_ = State::PASS;
    autoware_internal_planning_msgs::VelocityLimitClearCommand clear;
    clear.stamp = rosTimeToBuiltinTime(ros::Time::now()); clear.command = true; clear.sender = "surround_obstacle_checker";
    clear_command_pub_.publish(clear);
  }
  // Retain the source DebugNode's planning-factor lifecycle: each timer publishes a fresh
  // factor array; STOP contributes a zero-distance stop reason at the current ego pose.
  if (state_ == State::STOP) {
    autoware_internal_planning_msgs::SafetyFactorArray safety_factors;
    safety_factors.header.stamp = ros::Time::now(); safety_factors.header.frame_id = "map";
    if (nearest) {
      autoware_internal_planning_msgs::SafetyFactor safety_factor;
      safety_factor.is_safe = false;
      safety_factor.type = nearest->is_pointcloud ?
        autoware_internal_planning_msgs::SafetyFactor::POINTCLOUD :
        autoware_internal_planning_msgs::SafetyFactor::OBJECT;
      safety_factor.object_id = nearest->object_id;
      safety_factor.points.push_back(nearest->map_point);
      safety_factors.factors.push_back(safety_factor);
    }
    planning_factor_interface_->add(0.0, odometry->pose.pose,
      autoware_internal_planning_msgs::PlanningFactor::STOP, safety_factors);
  }
  planning_factor_interface_->publish();
  publishDebug(odometry, nearest);
  autoware_internal_debug_msgs::Float64Stamped elapsed;
  // The staged ROS1 debug message retained ros::Time while the planning-limit
  // messages retain builtin_interfaces::Time; preserve each generated ABI.
  elapsed.stamp = rosTimeToBuiltinTime(ros::Time::now());
  elapsed.data = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
  processing_time_pub_.publish(elapsed);
}
}  // namespace aw149::surround_obstacle_checker
