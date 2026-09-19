#ifndef MISSION_PLANNER__MISSION_PLANNER_INITIALIZATION_ROS1_HPP_
#define MISSION_PLANNER__MISSION_PLANNER_INITIALIZATION_ROS1_HPP_

#include <autoware/mission_planner_universe/mission_planner_plugin.hpp>
#include "arrival_checker.hpp"
#include <autoware_map_msgs/LaneletMapBin.h>
#include <autoware_adapi_v1_msgs/OperationModeState.h>
#include <autoware_planning_msgs/ClearRoute.h>
#include <autoware_planning_msgs/LaneletRoute.h>
#include <autoware_planning_msgs/RouteState.h>
#include <autoware_planning_msgs/SetLaneletRoute.h>
#include <autoware_planning_msgs/SetPreferredPrimitive.h>
#include <autoware_planning_msgs/SetWaypointRoute.h>
#include <autoware_planning_msgs/PoseWithUuidStamped.h>
#include <nav_msgs/Odometry.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tier4_planning_msgs/RerouteAvailability.h>
#include <visualization_msgs/MarkerArray.h>

#include <lanelet2_core/LaneletMap.h>

#include <memory>
#include <optional>
#include <string>

namespace autoware::mission_planner_universe
{
class MissionPlannerInitializationRos1
{
public:
  MissionPlannerInitializationRos1(ros::NodeHandle & node, ros::NodeHandle & private_node);

private:
  ros::NodeHandle & private_node_;
  pluginlib::ClassLoader<PlannerPlugin> plugin_loader_;
  boost::shared_ptr<PlannerPlugin> planner_;
  std::string map_frame_{"map"};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ArrivalChecker arrival_checker_;
  ros::Subscriber map_subscriber_;
  ros::Subscriber odometry_subscriber_;
  ros::Subscriber modified_goal_subscriber_;
  ros::Subscriber operation_mode_subscriber_;
  ros::Subscriber reroute_availability_subscriber_;
  ros::Publisher state_publisher_;
  ros::Publisher route_publisher_;
  ros::Publisher route_marker_publisher_;
  ros::ServiceServer clear_route_service_;
  ros::ServiceServer set_lanelet_route_service_;
  ros::ServiceServer set_preferred_primitive_service_;
  ros::ServiceServer set_waypoint_route_service_;
  autoware_planning_msgs::LaneletRoute current_route_;
  std::optional<autoware_planning_msgs::LaneletRoute> original_route_;
  nav_msgs::Odometry latest_odometry_;
  autoware_adapi_v1_msgs::OperationModeStateConstPtr operation_mode_state_;
  tier4_planning_msgs::RerouteAvailabilityConstPtr reroute_availability_;
  lanelet::LaneletMapPtr lanelet_map_;
  bool allow_reroute_in_autonomous_mode_{false};
  double reroute_time_threshold_{10.0};
  double minimum_reroute_length_{30.0};
  double goal_lanelet_transparency_{0.05};
  bool map_received_{false};
  bool odometry_received_{false};
  bool ready_published_{false};
  uint8_t route_state_{autoware_planning_msgs::RouteState::INITIALIZING};

  void on_map(const autoware_map_msgs::LaneletMapBinConstPtr & message);
  void on_odometry(const nav_msgs::OdometryConstPtr & message);
  void on_modified_goal(const autoware_planning_msgs::PoseWithUuidStampedConstPtr & message);
  void on_operation_mode(const autoware_adapi_v1_msgs::OperationModeStateConstPtr & message);
  void on_reroute_availability(const tier4_planning_msgs::RerouteAvailabilityConstPtr & message);
  void publish_ready_if_complete();
  void publish_state(uint8_t state);
  bool transform_goal_to_map(
    const geometry_msgs::Pose & input, const std_msgs::Header & header, geometry_msgs::Pose * output);
  bool is_autonomous_control_active() const;
  bool check_reroute_safety(
    const autoware_planning_msgs::LaneletRoute & original_route,
    const autoware_planning_msgs::LaneletRoute & target_route) const;
  void publish_route_outputs();
  bool on_clear_route(
    autoware_planning_msgs::ClearRoute::Request &, autoware_planning_msgs::ClearRoute::Response & response);
  bool on_set_lanelet_route(
    autoware_planning_msgs::SetLaneletRoute::Request & request,
    autoware_planning_msgs::SetLaneletRoute::Response & response);
  bool on_set_preferred_primitive(
    autoware_planning_msgs::SetPreferredPrimitive::Request & request,
    autoware_planning_msgs::SetPreferredPrimitive::Response & response);
  bool on_set_waypoint_route(
    autoware_planning_msgs::SetWaypointRoute::Request & request,
    autoware_planning_msgs::SetWaypointRoute::Response & response);
};
}  // namespace autoware::mission_planner_universe

#endif
