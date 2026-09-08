#include "mission_planner_initialization_ros1.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_map_msgs/LaneletMapBin.h>
#include <autoware_planning_msgs/RouteState.h>
#include <autoware_planning_msgs/ClearRoute.h>
#include <autoware_planning_msgs/SetLaneletRoute.h>
#include <autoware_planning_msgs/SetPreferredPrimitive.h>
#include <autoware_planning_msgs/PoseWithUuidStamped.h>
#include <autoware_adapi_v1_msgs/OperationModeState.h>
#include <tier4_planning_msgs/RerouteAvailability.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <string>

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: mission_planner_initialization_oracle <lanelet_map.osm>" << std::endl;
    return 64;
  }
  ros::init(argc, argv, "mission_planner_initialization_oracle");
  ros::NodeHandle node;
  ros::NodeHandle planner_node("/mission_planner");
  for (const auto & entry : {std::pair{"wheel_radius", 0.3}, {"wheel_width", 0.2},
                            {"wheel_base", 2.7}, {"wheel_tread", 1.5},
                            {"front_overhang", 0.9}, {"rear_overhang", 1.0},
                            {"left_overhang", 0.2}, {"right_overhang", 0.2},
                            {"vehicle_height", 1.6}, {"max_steer_angle", 0.6}}) {
    planner_node.setParam(entry.first, entry.second);
  }
  planner_node.setParam("allow_reroute_in_autonomous_mode", true);
  // The v61 oracle route shares only 8.919 m.  Keep the production defaults
  // untouched, but make this test's low-speed and high-speed safety cases
  // explicit and mutually distinguishable.
  planner_node.setParam("reroute_time_threshold", 1.0);
  planner_node.setParam("minimum_reroute_length", 1.0);

  bool observed_unset = false;
  bool observed_unset_after_clear = false;
  bool observed_set = false;
  int route_publish_count = 0;
  int route_marker_publish_count = 0;
  auto state_subscriber = node.subscribe<autoware_planning_msgs::RouteState>(
    "/mission_planner/state", 1,
    [&observed_unset, &observed_unset_after_clear, &observed_set](const autoware_planning_msgs::RouteStateConstPtr & state) {
      observed_unset = state->state == autoware_planning_msgs::RouteState::UNSET;
      observed_unset_after_clear = observed_set && state->state == autoware_planning_msgs::RouteState::UNSET;
      observed_set = observed_set || state->state == autoware_planning_msgs::RouteState::SET;
    });
  auto route_subscriber = node.subscribe<autoware_planning_msgs::LaneletRoute>(
    "/mission_planner/route", 10,
    [&route_publish_count](const autoware_planning_msgs::LaneletRouteConstPtr &) { ++route_publish_count; });
  auto route_marker_subscriber = node.subscribe<visualization_msgs::MarkerArray>(
    "/mission_planner/debug/route_marker", 10,
    [&route_marker_publish_count](const visualization_msgs::MarkerArrayConstPtr & markers) {
      if (!markers->markers.empty()) ++route_marker_publish_count;
    });
  auto map_publisher = planner_node.advertise<autoware_map_msgs::LaneletMapBin>("input/vector_map", 1, true);
  auto odometry_publisher = planner_node.advertise<nav_msgs::Odometry>("input/odometry", 1, true);
  auto modified_goal_publisher = planner_node.advertise<autoware_planning_msgs::PoseWithUuidStamped>("input/modified_goal", 1, true);
  auto operation_mode_publisher = planner_node.advertise<autoware_adapi_v1_msgs::OperationModeState>("input/operation_mode_state", 1, true);
  auto reroute_availability_publisher = planner_node.advertise<tier4_planning_msgs::RerouteAvailability>("input/reroute_availability", 1, true);

  autoware::mission_planner_universe::MissionPlannerInitializationRos1 planner(node, planner_node);
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(argv[1]);
  const auto map_message = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
  const auto first_lanelet = *map->laneletLayer.begin();
  const auto & centerline = first_lanelet.centerline();
  if (centerline.size() < 2) {
    std::cerr << "sample map first lanelet has no usable centerline" << std::endl;
    return 65;
  }
  const auto & start_point = centerline.front();
  const auto & end_point = centerline.back();
  const auto goal_x = (start_point.x() + end_point.x()) * 0.5;
  const auto goal_y = (start_point.y() + end_point.y()) * 0.5;
  const auto goal_z = (start_point.z() + end_point.z()) * 0.5;
  const auto yaw = std::atan2(end_point.y() - start_point.y(), end_point.x() - start_point.x());
  nav_msgs::Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.pose.pose.position.x = start_point.x();
  odometry.pose.pose.position.y = start_point.y();
  odometry.pose.pose.position.z = start_point.z();
  odometry.pose.pose.orientation.z = std::sin(yaw * 0.5);
  odometry.pose.pose.orientation.w = std::cos(yaw * 0.5);

  const auto deadline = ros::WallTime::now() + ros::WallDuration(70.0);
  while (ros::ok() && ros::WallTime::now() < deadline && !observed_unset) {
    map_publisher.publish(map_message);
    odometry_publisher.publish(odometry);
    ros::spinOnce();
    ros::WallDuration(0.05).sleep();
  }
  if (!observed_unset) {
    std::cerr << "MissionPlanner did not publish UNSET after valid map and odometry" << std::endl;
    return 1;
  }
  auto route_client = node.serviceClient<autoware_planning_msgs::SetLaneletRoute>(
    "/mission_planner/set_lanelet_route");
  if (!route_client.waitForExistence(ros::Duration(1.0))) {
    std::cerr << "set_lanelet_route service is absent" << std::endl;
    return 2;
  }
  autoware_planning_msgs::SetLaneletRoute route_service;
  route_service.request.header.frame_id = "map";
  route_service.request.goal_pose.orientation.w = 1.0;
  autoware_planning_msgs::LaneletSegment segment;
  autoware_planning_msgs::LaneletPrimitive primitive;
  primitive.id = map->laneletLayer.begin()->id();
  segment.primitives.push_back(primitive);
  segment.preferred_primitive = primitive;
  route_service.request.segments.push_back(segment);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  if (!route_client.call(route_service) || !route_service.response.status.success) {
    std::cerr << "map-frame set_lanelet_route failed" << std::endl;
    return 3;
  }
  const auto set_deadline = ros::WallTime::now() + ros::WallDuration(2.0);
  while (ros::ok() && ros::WallTime::now() < set_deadline && !observed_set) {
    ros::WallDuration(0.05).sleep();
  }
  if (!observed_set) {
    std::cerr << "route SET state was not published" << std::endl;
    return 4;
  }
  const auto marker_deadline = ros::WallTime::now() + ros::WallDuration(1.0);
  while (ros::ok() && ros::WallTime::now() < marker_deadline && route_marker_publish_count == 0) ros::WallDuration(0.02).sleep();
  if (route_marker_publish_count == 0) return 24;
  auto preferred_client = node.serviceClient<autoware_planning_msgs::SetPreferredPrimitive>(
    "/mission_planner/set_preferred_primitive");
  if (!preferred_client.waitForExistence(ros::Duration(1.0))) {
    std::cerr << "set_preferred_primitive service is absent" << std::endl;
    return 11;
  }
  autoware_planning_msgs::SetPreferredPrimitive preferred_service;
  preferred_service.request.uuid = route_service.request.uuid;
  preferred_service.request.preferred_primitives.push_back(primitive);
  if (!preferred_client.call(preferred_service) || !preferred_service.response.status.success) {
    std::cerr << "set_preferred_primitive valid primitive failed" << std::endl;
    return 12;
  }
  auto invalid_preferred_service = preferred_service;
  invalid_preferred_service.request.preferred_primitives.front().id = -1;
  if (!preferred_client.call(invalid_preferred_service) || invalid_preferred_service.response.status.success) {
    std::cerr << "set_preferred_primitive accepted a primitive outside the segment" << std::endl;
    return 13;
  }
  autoware_planning_msgs::SetPreferredPrimitive reset_preferred_service;
  reset_preferred_service.request.uuid = route_service.request.uuid;
  reset_preferred_service.request.reset = true;
  if (!preferred_client.call(reset_preferred_service) || !reset_preferred_service.response.status.success) {
    std::cerr << "set_preferred_primitive reset failed" << std::endl;
    return 14;
  }
  tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped map_from_odom;
  map_from_odom.header.stamp = ros::Time::now();
  map_from_odom.header.frame_id = "map";
  map_from_odom.child_frame_id = "odom";
  map_from_odom.transform.translation.x = 1.0;
  map_from_odom.transform.rotation.w = 1.0;
  static_broadcaster.sendTransform(map_from_odom);
  ros::WallDuration(0.1).sleep();
  auto transformed_route_service = route_service;
  transformed_route_service.request.header.frame_id = "odom";
  transformed_route_service.request.goal_pose.position.x = 2.0;
  if (!route_client.call(transformed_route_service) || transformed_route_service.response.status.success) {
    std::cerr << "reroute without operation mode state was accepted" << std::endl;
    return 16;
  }
  const auto operation_mode_deadline = ros::WallTime::now() + ros::WallDuration(1.0);
  while (ros::ok() && ros::WallTime::now() < operation_mode_deadline && operation_mode_publisher.getNumSubscribers() == 0) {
    ros::WallDuration(0.02).sleep();
  }
  if (operation_mode_publisher.getNumSubscribers() == 0) {
    std::cerr << "operation mode subscriber is absent" << std::endl;
    return 17;
  }
  autoware_adapi_v1_msgs::OperationModeState local_mode;
  local_mode.mode = autoware_adapi_v1_msgs::OperationModeState::LOCAL;
  operation_mode_publisher.publish(local_mode);
  ros::WallDuration(0.05).sleep();
  if (!route_client.call(transformed_route_service) || !transformed_route_service.response.status.success) {
    std::cerr << "TF-transformed set_lanelet_route failed" << std::endl;
    return 5;
  }
  autoware_adapi_v1_msgs::OperationModeState autonomous_mode;
  autonomous_mode.mode = autoware_adapi_v1_msgs::OperationModeState::AUTONOMOUS;
  autonomous_mode.is_autoware_control_enabled = true;
  operation_mode_publisher.publish(autonomous_mode);
  ros::WallDuration(0.05).sleep();
  if (!route_client.call(transformed_route_service) || transformed_route_service.response.status.success) {
    std::cerr << "autonomous reroute without availability was accepted" << std::endl;
    return 18;
  }
  const auto availability_deadline = ros::WallTime::now() + ros::WallDuration(1.0);
  while (ros::ok() && ros::WallTime::now() < availability_deadline && reroute_availability_publisher.getNumSubscribers() == 0) {
    ros::WallDuration(0.02).sleep();
  }
  if (reroute_availability_publisher.getNumSubscribers() == 0) {
    std::cerr << "reroute availability subscriber is absent" << std::endl;
    return 19;
  }
  tier4_planning_msgs::RerouteAvailability availability;
  availability.availability = true;
  reroute_availability_publisher.publish(availability);
  ros::WallDuration(0.05).sleep();
  if (!route_client.call(transformed_route_service) || !transformed_route_service.response.status.success) {
    return 20;
  }
  odometry.twist.twist.linear.x = 100.0;
  odometry_publisher.publish(odometry);
  ros::WallDuration(0.05).sleep();
  if (!route_client.call(transformed_route_service) || transformed_route_service.response.status.success) return 21;
  odometry.twist.twist.linear.x = 1.0;
  odometry_publisher.publish(odometry);
  ros::WallDuration(0.05).sleep();
  if (!route_client.call(transformed_route_service) || !transformed_route_service.response.status.success) return 22;
  ros::WallDuration(0.05).sleep();
  auto waypoint_client = node.serviceClient<autoware_planning_msgs::SetWaypointRoute>(
    "/mission_planner/set_waypoint_route");
  if (!waypoint_client.waitForExistence(ros::Duration(1.0))) {
    std::cerr << "set_waypoint_route service is absent" << std::endl;
    return 6;
  }
  autoware_planning_msgs::SetWaypointRoute waypoint_service;
  waypoint_service.request.header.frame_id = "map";
  waypoint_service.request.goal_pose.position.x = goal_x;
  waypoint_service.request.goal_pose.position.y = goal_y;
  waypoint_service.request.goal_pose.position.z = goal_z;
  waypoint_service.request.goal_pose.orientation.z = std::sin(yaw * 0.5);
  waypoint_service.request.goal_pose.orientation.w = std::cos(yaw * 0.5);
  odometry.twist.twist.linear.x = 100.0;
  odometry_publisher.publish(odometry);
  ros::WallDuration(0.05).sleep();
  if (!waypoint_client.call(waypoint_service) || waypoint_service.response.status.success || waypoint_service.response.status.message != "New route is not safe. Reroute failed.") return 23;
  odometry.twist.twist.linear.x = 0.0;
  odometry_publisher.publish(odometry);
  ros::WallDuration(0.05).sleep();
  operation_mode_publisher.publish(local_mode);
  ros::WallDuration(0.05).sleep();
  if (!waypoint_client.call(waypoint_service) || !waypoint_service.response.status.success) {
    std::cerr << "map-native set_waypoint_route failed: "
              << waypoint_service.response.status.message << std::endl;
    return 7;
  }
  ros::WallDuration(0.1).sleep();
  const auto route_publish_count_before_modified_goal = route_publish_count;
  autoware_planning_msgs::PoseWithUuidStamped modified_goal;
  modified_goal.header = waypoint_service.request.header;
  modified_goal.pose = waypoint_service.request.goal_pose;
  modified_goal.uuid = waypoint_service.request.uuid;
  modified_goal_publisher.publish(modified_goal);
  const auto reroute_deadline = ros::WallTime::now() + ros::WallDuration(2.0);
  while (
    ros::ok() && ros::WallTime::now() < reroute_deadline &&
    route_publish_count <= route_publish_count_before_modified_goal) {
    ros::WallDuration(0.05).sleep();
  }
  if (route_publish_count <= route_publish_count_before_modified_goal) {
    std::cerr << "modified_goal did not publish a replacement route" << std::endl;
    return 15;
  }
  auto clear_client = node.serviceClient<autoware_planning_msgs::ClearRoute>(
    "/mission_planner/clear_route");
  if (!clear_client.waitForExistence(ros::Duration(1.0))) {
    std::cerr << "clear_route service is absent" << std::endl;
    return 8;
  }
  autoware_planning_msgs::ClearRoute clear_service;
  if (!clear_client.call(clear_service) || !clear_service.response.status.success) {
    std::cerr << "clear_route failed" << std::endl;
    return 9;
  }
  const auto clear_deadline = ros::WallTime::now() + ros::WallDuration(2.0);
  while (ros::ok() && ros::WallTime::now() < clear_deadline && !observed_unset_after_clear) {
    ros::WallDuration(0.05).sleep();
  }
  if (!observed_unset_after_clear) {
    std::cerr << "UNSET state was not restored after clear_route" << std::endl;
    return 10;
  }
  std::cout << "mission_planner_initialization_and_route_oracle=PASS" << std::endl;
  return 0;
}
