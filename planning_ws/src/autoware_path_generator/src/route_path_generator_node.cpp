#include <autoware/path_generator/route_path_generator_node.hpp>

namespace autoware::path_generator
{
RoutePathGeneratorNode::RoutePathGeneratorNode(ros::NodeHandle node) : node_(std::move(node))
{
  node_.param("ego_nearest_dist_threshold", parameters_.ego_nearest_dist_threshold, parameters_.ego_nearest_dist_threshold);
  node_.param("ego_nearest_yaw_threshold", parameters_.ego_nearest_yaw_threshold, parameters_.ego_nearest_yaw_threshold);
  node_.param("forward_length", parameters_.forward_length, parameters_.forward_length);
  node_.param("backward_length", parameters_.backward_length, parameters_.backward_length);
  double planning_hz = 10.0;
  node_.param("planning_hz", planning_hz, planning_hz);
  if (planning_hz <= 0.0) planning_hz = 10.0;
  map_subscriber_ = node_.subscribe("input/vector_map", 1, &RoutePathGeneratorNode::onMap, this);
  route_subscriber_ = node_.subscribe("input/route", 1, &RoutePathGeneratorNode::onRoute, this);
  odometry_subscriber_ = node_.subscribe("input/odometry", 1, &RoutePathGeneratorNode::onOdometry, this);
  path_publisher_ = node_.advertise<autoware_internal_planning_msgs::PathWithLaneId>("output/path", 1);
  planning_timer_ = node_.createTimer(ros::Duration(1.0 / planning_hz), [this](const ros::TimerEvent &) { publishIfReady(); });
}
void RoutePathGeneratorNode::onMap(const autoware_map_msgs::LaneletMapBin::ConstPtr & message) { map_ = message; has_initial_pose_ = false; }
void RoutePathGeneratorNode::onRoute(const autoware_planning_msgs::LaneletRoute::ConstPtr & message) { route_ = message; has_initial_pose_ = false; }
void RoutePathGeneratorNode::onOdometry(const nav_msgs::Odometry::ConstPtr & message)
{
  current_pose_ = message->pose.pose;
  has_current_pose_ = true;
  if (!has_initial_pose_) { initial_pose_ = current_pose_; has_initial_pose_ = true; }
}
void RoutePathGeneratorNode::publishIfReady()
{
  if (!map_ || !route_ || !has_initial_pose_ || !has_current_pose_) return;
  const auto path = generateRoutePath(*map_, *route_, initial_pose_, current_pose_, parameters_);
  if (path) {
    auto output = *path;
    output.header.stamp = ros::Time::now();
    path_publisher_.publish(output);
  }
}
}  // namespace autoware::path_generator
