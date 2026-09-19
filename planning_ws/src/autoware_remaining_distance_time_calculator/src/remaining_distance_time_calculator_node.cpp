#include "autoware/remaining_distance_time_calculator/node.hpp"

#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/lanelet2_utils/nn_search.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <numeric>

namespace autoware::remaining_distance_time_calculator
{
RemainingDistanceTimeCalculatorNode::RemainingDistanceTimeCalculatorNode(ros::NodeHandle node) : node_(std::move(node))
{
  double update_rate = 10.0;
  node_.param("update_rate", update_rate, update_rate);
  map_sub_ = node_.subscribe("input/map", 1, &RemainingDistanceTimeCalculatorNode::on_map, this);
  route_sub_ = node_.subscribe("input/route", 1, &RemainingDistanceTimeCalculatorNode::on_route, this);
  odometry_sub_ = node_.subscribe("input/odometry", 1, &RemainingDistanceTimeCalculatorNode::on_odometry, this);
  velocity_limit_sub_ = node_.subscribe("input/velocity_limit", 1, &RemainingDistanceTimeCalculatorNode::on_velocity_limit, this);
  scenario_sub_ = node_.subscribe("input/scenario", 1, &RemainingDistanceTimeCalculatorNode::on_scenario, this);
  output_pub_ = node_.advertise<autoware_internal_msgs::MissionRemainingDistanceTime>("output/mission_remaining_distance_time", 10);
  timer_ = node_.createTimer(ros::Duration(1.0 / std::max(update_rate, 1e-3)), &RemainingDistanceTimeCalculatorNode::on_timer, this);
}

void RemainingDistanceTimeCalculatorNode::on_map(const autoware_map_msgs::LaneletMapBin::ConstPtr & message)
{
  route_handler_.setMap(*message);
  map_received_ = route_handler_.isMapMsgReady();
  if (map_received_ && route_) route_handler_.setRoute(*route_);
}
void RemainingDistanceTimeCalculatorNode::on_route(const autoware_planning_msgs::LaneletRoute::ConstPtr & message)
{
  route_ = message;
  if (map_received_) route_handler_.setRoute(*route_);
}
void RemainingDistanceTimeCalculatorNode::on_odometry(const nav_msgs::Odometry::ConstPtr & message)
{
  current_pose_ = message->pose.pose;
  odometry_received_ = true;
}
void RemainingDistanceTimeCalculatorNode::on_velocity_limit(const autoware_internal_planning_msgs::VelocityLimit::ConstPtr & message)
{
  if (message->max_velocity > 1e-5) velocity_limit_ = message->max_velocity;
}
void RemainingDistanceTimeCalculatorNode::on_scenario(const autoware_internal_planning_msgs::Scenario::ConstPtr & message) { scenario_ = message; }

bool RemainingDistanceTimeCalculatorNode::calculate_remaining_distance(double * distance) const
{
  if (!route_handler_.isHandlerReady() || !odometry_received_) return false;
  // RouteHandler deliberately hides its raw route-lanelet cache in ROS1.  Its
  // public preferred-lane sequence is the route-order surface used by planners.
  const auto lanes = route_handler_.getPreferredLanelets();
  if (lanes.empty()) return false;
  lanelet::ConstLanelet current;
  if (!route_handler_.getClosestLaneletWithinRoute(current_pose_, &current)) return false;
  const auto current_it = std::find_if(lanes.begin(), lanes.end(), [&](const auto & lane) { return lane.id() == current.id(); });
  if (current_it == lanes.end()) return false;
  const auto goal = route_handler_.getGoalPose();
  lanelet::ConstLanelet goal_lane;
  if (!route_handler_.getClosestLaneletWithinRoute(goal, &goal_lane)) return false;
  const double current_s = autoware::experimental::lanelet2_utils::get_arc_coordinates({current}, current_pose_).length;
  const double goal_s = autoware::experimental::lanelet2_utils::get_arc_coordinates({goal_lane}, goal).length;
  if (current.id() == goal_lane.id()) { *distance = std::max(goal_s - current_s, 0.0); return true; }
  double value = lanelet::geometry::length2d(current) - current_s + goal_s;
  for (auto it = std::next(current_it); it != lanes.end() && it->id() != goal_lane.id(); ++it) value += lanelet::geometry::length2d(*it);
  *distance = std::max(value, 0.0);
  return true;
}
void RemainingDistanceTimeCalculatorNode::publish(double distance, double time) const
{
  autoware_internal_msgs::MissionRemainingDistanceTime output;
  output.remaining_distance = distance;
  output.remaining_time = time;
  output_pub_.publish(output);
}
void RemainingDistanceTimeCalculatorNode::on_timer(const ros::TimerEvent &)
{
  if (!scenario_) return;
  if (scenario_->current_scenario == autoware_internal_planning_msgs::Scenario::PARKING) { publish(0.0, 0.0); return; }
  double distance = 0.0;
  if (calculate_remaining_distance(&distance)) publish(distance, velocity_limit_ > 0.0 ? distance / velocity_limit_ : 0.0);
}
}  // namespace autoware::remaining_distance_time_calculator
