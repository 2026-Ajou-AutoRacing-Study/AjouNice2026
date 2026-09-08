#include "autoware/scenario_selector/node.hpp"

#include <autoware_lanelet2_extension/utility/query.hpp>

#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <boost/make_shared.hpp>

#include <cmath>
#include <utility>
#include <vector>

namespace autoware::scenario_selector
{
namespace
{
constexpr double kLaneStoppingTimeoutS = 5.0;
constexpr double kEmptyParkingTrajectoryTimeoutS = 3.0;

std::shared_ptr<lanelet::ConstPolygon3d> findNearestParkingLot(
  const lanelet::LaneletMapPtr & map, const lanelet::BasicPoint2d & position)
{
  auto parking_lot = std::make_shared<lanelet::ConstPolygon3d>();
  if (lanelet::utils::query::getLinkedParkingLot(position, map, parking_lot.get())) {
    return parking_lot;
  }
  return {};
}

bool isInLane(const lanelet::LaneletMapPtr & map, const geometry_msgs::Point & position)
{
  const lanelet::BasicPoint2d point(position.x, position.y);
  const auto nearest = lanelet::geometry::findNearest(map->laneletLayer, point, 1);
  return !nearest.empty() && nearest.front().first < 0.01;
}

bool isInParkingLot(const lanelet::LaneletMapPtr & map, const geometry_msgs::Pose & pose)
{
  const auto & point = pose.position;
  const lanelet::Point3d search(lanelet::InvalId, point.x, point.y, point.z);
  const auto parking_lot = findNearestParkingLot(map, search.basicPoint2d());
  return parking_lot && lanelet::geometry::within(search, parking_lot->basicPolygon());
}

bool isAlongLane(
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const geometry_msgs::Pose & pose)
{
  lanelet::ConstLanelet closest;
  if (!route_handler->getClosestLaneletWithConstrainsWithinRoute(pose, &closest, 0.0, M_PI_4)) {
    return false;
  }
  return lanelet::geometry::distanceToCenterline2d(
           closest, lanelet::BasicPoint2d(pose.position.x, pose.position.y)) < 1.0;
}

bool isNearTrajectoryEnd(
  const autoware_planning_msgs::Trajectory::ConstPtr & trajectory,
  const geometry_msgs::Pose & pose, const double threshold)
{
  if (!trajectory || trajectory->points.empty()) {
    return false;
  }
  const auto & end = trajectory->points.back().pose.position;
  return std::hypot(pose.position.x - end.x, pose.position.y - end.y) < threshold;
}

bool isStopped(
  const std::deque<geometry_msgs::TwistStamped::ConstPtr> & twists, const double threshold)
{
  for (const auto & twist : twists) {
    if (std::abs(twist->twist.linear.x) > threshold) {
      return false;
    }
  }
  return true;
}
}  // namespace

ScenarioSelectorNode::ScenarioSelectorNode(const ros::NodeHandle & node)
: node_(node), private_node_("~"), current_scenario_(autoware_internal_planning_msgs::Scenario::EMPTY)
{
  private_node_.param("update_rate", update_rate_, 10.0);
  private_node_.param("th_max_message_delay_sec", th_max_message_delay_sec_, 1.0);
  private_node_.param("th_arrived_distance_m", th_arrived_distance_m_, 1.0);
  private_node_.param("th_stopped_time_sec", th_stopped_time_sec_, 1.0);
  private_node_.param("th_stopped_velocity_mps", th_stopped_velocity_mps_, 0.01);
  private_node_.param("enable_mode_switching", enable_mode_switching_, true);
  if (update_rate_ <= 0.0) {
    throw std::invalid_argument("update_rate must be positive");
  }

  map_sub_ = node_.subscribe("input/lanelet_map", 1, &ScenarioSelectorNode::onMap, this);
  route_sub_ = node_.subscribe("input/route", 1, &ScenarioSelectorNode::onRoute, this);
  odom_sub_ = node_.subscribe("input/odometry", 100, &ScenarioSelectorNode::onOdom, this);
  lane_driving_sub_ = node_.subscribe(
    "input/lane_driving/trajectory", 1, &ScenarioSelectorNode::onLaneDrivingTrajectory, this);
  parking_sub_ = node_.subscribe(
    "input/parking/trajectory", 1, &ScenarioSelectorNode::onParkingTrajectory, this);
  parking_completed_sub_ = node_.subscribe(
    "is_parking_completed", 1, &ScenarioSelectorNode::onParkingCompleted, this);
  operation_mode_sub_ = node_.subscribe(
    "input/operation_mode_state", 1, &ScenarioSelectorNode::onOperationMode, this);
  scenario_pub_ = node_.advertise<autoware_internal_planning_msgs::Scenario>("output/scenario", 1);
  trajectory_pub_ = node_.advertise<autoware_planning_msgs::Trajectory>("output/trajectory", 1);
  processing_time_pub_ = private_node_.advertise<autoware_internal_debug_msgs::Float64Stamped>(
    "debug/processing_time_ms", 1);
  timer_ = node_.createTimer(ros::Duration(1.0 / update_rate_), &ScenarioSelectorNode::onTimer, this);
}

void ScenarioSelectorNode::onMap(const autoware_map_msgs::LaneletMapBin::ConstPtr & message)
{
  route_handler_ = std::make_shared<autoware::route_handler::RouteHandler>(*message);
}

void ScenarioSelectorNode::onRoute(const autoware_planning_msgs::LaneletRoute::ConstPtr & message)
{
  if (!route_handler_ || route_handler_->getRouteUuid() != message->uuid) {
    current_scenario_ = autoware_internal_planning_msgs::Scenario::EMPTY;
  }
  route_ = message;
}

void ScenarioSelectorNode::onOdom(const nav_msgs::Odometry::ConstPtr & message)
{
  pending_odometry_.push_back(message);
}

void ScenarioSelectorNode::applyOdom(const nav_msgs::Odometry::ConstPtr & message)
{
  current_pose_ = message;
  auto twist = boost::make_shared<geometry_msgs::TwistStamped>();
  twist->header = message->header;
  twist->twist = message->twist.twist;
  twist_ = twist;
  twist_buffer_.push_back(twist);
  while (!twist_buffer_.empty() &&
    (message->header.stamp - twist_buffer_.front()->header.stamp).toSec() >= th_stopped_time_sec_) {
    twist_buffer_.pop_front();
  }
}

void ScenarioSelectorNode::onLaneDrivingTrajectory(
  const autoware_planning_msgs::Trajectory::ConstPtr & message)
{
  lane_driving_trajectory_ = message;
  if (current_scenario_ == autoware_internal_planning_msgs::Scenario::LANEDRIVING) {
    publishTrajectory(message);
  }
}

void ScenarioSelectorNode::onParkingTrajectory(const autoware_planning_msgs::Trajectory::ConstPtr & message)
{
  parking_trajectory_ = message;
  if (current_scenario_ == autoware_internal_planning_msgs::Scenario::PARKING) {
    publishTrajectory(message);
  }
}

void ScenarioSelectorNode::onParkingCompleted(const std_msgs::Bool::ConstPtr & message)
{
  pending_parking_completed_ = message->data;
}

void ScenarioSelectorNode::onOperationMode(
  const autoware_adapi_v1_msgs::OperationModeState::ConstPtr & message)
{
  pending_operation_mode_state_ = message;
}

void ScenarioSelectorNode::updatePolledData()
{
  if (pending_parking_completed_) {
    is_parking_completed_ = *pending_parking_completed_;
    pending_parking_completed_.reset();
  }
  while (!pending_odometry_.empty()) {
    applyOdom(pending_odometry_.front());
    pending_odometry_.pop_front();
  }
  if (pending_operation_mode_state_) {
    operation_mode_state_ = pending_operation_mode_state_;
    pending_operation_mode_state_.reset();
  }
}

bool ScenarioSelectorNode::isDataReady()
{
  if (!current_pose_ || !route_handler_ || !route_ || !twist_ || !operation_mode_state_) {
    return false;
  }
  route_handler_->setRoute(*route_);
  return route_handler_->isHandlerReady();
}

autoware_planning_msgs::Trajectory::ConstPtr ScenarioSelectorNode::getScenarioTrajectory(
  const std::string & scenario) const
{
  if (scenario == autoware_internal_planning_msgs::Scenario::PARKING) {
    return parking_trajectory_;
  }
  return lane_driving_trajectory_;
}

std::string ScenarioSelectorNode::selectScenarioByPosition()
{
  const auto map = route_handler_->getLaneletMapPtr();
  const bool in_lane = isInLane(map, current_pose_->pose.pose.position);
  const bool goal_in_lane = isInLane(map, route_->goal_pose.position);
  const bool in_parking_lot = isInParkingLot(map, current_pose_->pose.pose);
  if (current_scenario_ == autoware_internal_planning_msgs::Scenario::EMPTY) {
    if (in_lane && goal_in_lane) return autoware_internal_planning_msgs::Scenario::LANEDRIVING;
    if (in_parking_lot) return autoware_internal_planning_msgs::Scenario::PARKING;
    return autoware_internal_planning_msgs::Scenario::LANEDRIVING;
  }
  if (current_scenario_ == autoware_internal_planning_msgs::Scenario::LANEDRIVING &&
      in_parking_lot && !goal_in_lane) {
    return autoware_internal_planning_msgs::Scenario::PARKING;
  }
  if (current_scenario_ == autoware_internal_planning_msgs::Scenario::PARKING &&
      is_parking_completed_ && in_lane) {
    is_parking_completed_ = false;
    return autoware_internal_planning_msgs::Scenario::LANEDRIVING;
  }
  return current_scenario_;
}

bool ScenarioSelectorNode::isAutonomous() const
{
  return operation_mode_state_->mode == autoware_adapi_v1_msgs::OperationModeState::AUTONOMOUS &&
         operation_mode_state_->is_autoware_control_enabled;
}

bool ScenarioSelectorNode::isEmptyParkingTrajectory() const
{
  return parking_trajectory_ && parking_trajectory_->points.size() <= 1;
}

bool ScenarioSelectorNode::isSwitchToParking(const bool stopped)
{
  const auto map = route_handler_->getLaneletMapPtr();
  if (!stopped || !isAutonomous() || !isInParkingLot(map, current_pose_->pose.pose) ||
      isInLane(map, route_->goal_pose.position)) {
    lane_driving_stop_time_.reset();
    return false;
  }
  if (!lane_driving_stop_time_) {
    lane_driving_stop_time_ = ros::Time::now();
    return false;
  }
  return (ros::Time::now() - *lane_driving_stop_time_).toSec() > kLaneStoppingTimeoutS;
}

bool ScenarioSelectorNode::isSwitchToLaneDriving()
{
  if (!isEmptyParkingTrajectory() || !isAlongLane(route_handler_, current_pose_->pose.pose)) {
    empty_parking_trajectory_time_.reset();
    return false;
  }
  if (!empty_parking_trajectory_time_) {
    empty_parking_trajectory_time_ = ros::Time::now();
    return false;
  }
  return (ros::Time::now() - *empty_parking_trajectory_time_).toSec() >
         kEmptyParkingTrajectoryTimeoutS;
}

void ScenarioSelectorNode::updateCurrentScenario()
{
  const auto previous = current_scenario_;
  const bool stopped = isStopped(twist_buffer_, th_stopped_velocity_mps_);
  if (isNearTrajectoryEnd(getScenarioTrajectory(current_scenario_), current_pose_->pose.pose,
      th_arrived_distance_m_) && stopped) {
    current_scenario_ = selectScenarioByPosition();
  }
  if (enable_mode_switching_) {
    if (current_scenario_ == autoware_internal_planning_msgs::Scenario::LANEDRIVING &&
        isSwitchToParking(stopped)) {
      current_scenario_ = autoware_internal_planning_msgs::Scenario::PARKING;
    } else if (current_scenario_ == autoware_internal_planning_msgs::Scenario::PARKING &&
               isSwitchToLaneDriving()) {
      current_scenario_ = autoware_internal_planning_msgs::Scenario::LANEDRIVING;
    }
  }
  if (current_scenario_ != previous) {
    lane_driving_stop_time_.reset();
    empty_parking_trajectory_time_.reset();
  }
}

void ScenarioSelectorNode::onTimer(const ros::TimerEvent &)
{
  const ros::WallTime started = ros::WallTime::now();
  updatePolledData();
  if (!isDataReady()) {
    return;
  }
  if (current_scenario_ == autoware_internal_planning_msgs::Scenario::EMPTY) {
    current_scenario_ = selectScenarioByPosition();
  }
  updateCurrentScenario();
  autoware_internal_planning_msgs::Scenario scenario;
  scenario.current_scenario = current_scenario_;
  if (current_scenario_ == autoware_internal_planning_msgs::Scenario::PARKING) {
    scenario.activating_scenarios.push_back(current_scenario_);
  }
  scenario_pub_.publish(scenario);
  autoware_internal_debug_msgs::Float64Stamped processing_time;
  const auto stamp = ros::Time::now();
  // Float64Stamped preserves the ROS2 builtin_interfaces Time layout in the
  // staged ROS1 message provider; copy the wire fields rather than assigning
  // a roscpp Time object directly.
  processing_time.stamp.sec = stamp.sec;
  processing_time.stamp.nanosec = stamp.nsec;
  processing_time.data = (ros::WallTime::now() - started).toSec() * 1000.0;
  processing_time_pub_.publish(processing_time);
}

void ScenarioSelectorNode::publishTrajectory(const autoware_planning_msgs::Trajectory::ConstPtr & message)
{
  if ((ros::Time::now() - message->header.stamp).toSec() <= th_max_message_delay_sec_) {
    trajectory_pub_.publish(*message);
  }
}
}  // namespace autoware::scenario_selector
